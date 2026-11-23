# Run `pip install "gymnasium[classic-control]"`

import gymnasium as gym
import torch
from torch import nn
from torch.distributions import Categorical
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from dataclasses import dataclass, fields
import copy

ALGORITHM = "PPO (clipped), GAE, parallel envs, minibatch epochs"

@dataclass
class Config:
    env_id: str = "CartPole-v1"

    # Data collection
    num_envs: int = 8                 # environments stepped together
    rollout_steps: int = 128          # steps per env between updates -> batch = 8 × 128 = 1024
    total_timesteps: int = 300_000    # summed over all envs

    # Returns / advantages
    discount_factor: float = 0.99
    gae_lambda: float = 0.95

    # PPO update
    update_epochs: int = 4            # passes over the same batch
    num_minibatches: int = 4          # minibatch = 1024 / 4 = 256
    clip_coef: float = 0.2            # probability ratio kept in [0.8, 1.2]
    normalize_advantages: bool = True

    # Optimisation
    actor_learning_rate: float = 3e-4
    critic_learning_rate: float = 1e-3
    lr_end_factor: float = 0.1        # both lrs decay linearly to this fraction (1.0 = no decay)
    entropy_coef: float = 0.01
    max_grad_norm: float = 0.5

    # Evaluation / logging
    evaluation_episodes: int = 20     # per action-selection mode, without rendering
    render_episodes: int = 1
    moving_avg_window: int = 50
    print_every: int = 10             # in updates


def compute_gae(rewards, values, next_values, terminations, dones, gamma, lam):
    """GAE over a [T, N] rollout. Episodes may end anywhere inside it.

    terminations: pole fell        -> no bootstrap from next state
    dones:        terminated or truncated -> advantage chain is cut at the episode boundary
    """
    advantages = torch.zeros_like(rewards)
    gae = torch.zeros(rewards.shape[1])
    for t in reversed(range(rewards.shape[0])):
        delta = rewards[t] + gamma * next_values[t] * (1.0 - terminations[t]) - values[t]
        gae = delta + gamma * lam * (1.0 - dones[t]) * gae
        advantages[t] = gae
    return advantages, advantages + values   # advantages, critic targets

class PPOAgent:
    def __init__(self, config: Config, obs_dim: int, n_actions: int):
        self.config = config

        self.actor = nn.Sequential(
            nn.Linear(obs_dim, 64), nn.Tanh(),
            nn.Linear(64, 64), nn.Tanh(),
            nn.Linear(64, n_actions),        # logits
        )
        self.critic = nn.Sequential(
            nn.Linear(obs_dim, 64), nn.Tanh(),
            nn.Linear(64, 64), nn.Tanh(),
            nn.Linear(64, 1),                # state value
        )
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=config.actor_learning_rate)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=config.critic_learning_rate)

    def get_action(self, obs: np.ndarray, deterministic: bool = False) -> torch.Tensor:
        state = torch.as_tensor(obs, dtype=torch.float32)
        with torch.no_grad():
            logits = self.actor(state)
            if deterministic:
                return logits.argmax()
            return Categorical(logits=logits).sample()

    def collect_rollout(self, envs: list[gym.Env]) -> dict[str, torch.Tensor]:
        """Step all envs for rollout_steps. Continues from where the previous rollout stopped."""
        T, N = self.config.rollout_steps, len(envs)
        obs_dim = self.obs.shape[1]

        buf = {
            "obs":          torch.zeros(T, N, obs_dim),
            "next_obs":     torch.zeros(T, N, obs_dim),   # true next state, even if the episode ended
            "actions":      torch.zeros(T, N, dtype=torch.int64),
            "logprobs":     torch.zeros(T, N),
            "rewards":      torch.zeros(T, N),
            "terminations": torch.zeros(T, N),
            "dones":        torch.zeros(T, N),
        }

        for t in range(T):
            obs = torch.as_tensor(self.obs, dtype=torch.float32)
            with torch.no_grad():
                dist = Categorical(logits=self.actor(obs))   # ONE forward pass for all envs
                actions = dist.sample()
                logprobs = dist.log_prob(actions)

            next_obs = np.empty_like(self.obs)
            for i, env in enumerate(envs):
                o, r, terminated, truncated, _ = env.step(actions[i].item())
                buf["next_obs"][t, i] = torch.as_tensor(o)
                buf["rewards"][t, i] = r
                buf["terminations"][t, i] = float(terminated)
                buf["dones"][t, i] = float(terminated or truncated)

                self.running_returns[i] += r
                if terminated or truncated:
                    self.episode_returns.append(float(self.running_returns[i]))
                    self.running_returns[i] = 0.0
                    o, _ = env.reset()
                next_obs[i] = o

            buf["obs"][t] = obs
            buf["actions"][t] = actions
            buf["logprobs"][t] = logprobs
            self.obs = next_obs

        return buf


    def update(self, buf: dict[str, torch.Tensor]) -> dict[str, float]:
        c = self.config

        with torch.no_grad():
            values = self.critic(buf["obs"]).squeeze(-1)            # [T, N]
            next_values = self.critic(buf["next_obs"]).squeeze(-1)
            advantages, returns = compute_gae(
                buf["rewards"], values, next_values,
                buf["terminations"], buf["dones"],
                c.discount_factor, c.gae_lambda,
            )

        # Flatten [T, N] -> [T*N]: from here on, order no longer matters
        obs = buf["obs"].reshape(-1, buf["obs"].shape[-1])
        actions = buf["actions"].reshape(-1)
        old_logprobs = buf["logprobs"].reshape(-1)
        advantages = advantages.reshape(-1)
        returns = returns.reshape(-1)

        batch_size = obs.shape[0]
        minibatch_size = batch_size // c.num_minibatches
        stats = {"approx_kl": [], "clip_frac": [], "entropy": []}

        for _ in range(c.update_epochs):
            perm = torch.randperm(batch_size)
            for start in range(0, batch_size, minibatch_size):
                idx = perm[start:start + minibatch_size]

                dist = Categorical(logits=self.actor(obs[idx]))
                new_logprobs = dist.log_prob(actions[idx])
                entropy = dist.entropy().mean()

                log_ratio = new_logprobs - old_logprobs[idx]
                ratio = log_ratio.exp()                     # π_new(a|s) / π_old(a|s)

                adv = advantages[idx]
                if c.normalize_advantages:
                    adv = (adv - adv.mean()) / (adv.std() + 1e-8)

                # Clipped surrogate: keep the more pessimistic of the two
                unclipped = ratio * adv
                clipped = torch.clamp(ratio, 1 - c.clip_coef, 1 + c.clip_coef) * adv
                actor_loss = -torch.min(unclipped, clipped).mean() - c.entropy_coef * entropy

                critic_loss = ((returns[idx] - self.critic(obs[idx]).squeeze(-1)) ** 2).mean()

                self.actor_optimizer.zero_grad()
                actor_loss.backward()
                nn.utils.clip_grad_norm_(self.actor.parameters(), c.max_grad_norm)
                self.actor_optimizer.step()

                self.critic_optimizer.zero_grad()
                critic_loss.backward()
                nn.utils.clip_grad_norm_(self.critic.parameters(), c.max_grad_norm)
                self.critic_optimizer.step()

                with torch.no_grad():
                    stats["approx_kl"].append(((ratio - 1) - log_ratio).mean().item())
                    stats["clip_frac"].append(((ratio - 1).abs() > c.clip_coef).float().mean().item())
                    stats["entropy"].append(entropy.item())

        return {k: float(np.mean(v)) for k, v in stats.items()}


    def train(self, envs: list[gym.Env]) -> list[float]:
        c = self.config
        batch_size = c.num_envs * c.rollout_steps
        num_updates = c.total_timesteps // batch_size

        self.obs = np.stack([env.reset()[0] for env in envs])
        self.running_returns = np.zeros(len(envs))
        self.episode_returns: list[float] = []

        self.best_moving_avg = -float("inf")
        self.best_actor_state = None
        self.best_episode = None

        schedulers = [
            torch.optim.lr_scheduler.LinearLR(
                opt, start_factor=1.0, end_factor=c.lr_end_factor, total_iters=num_updates,
            )
            for opt in (self.actor_optimizer, self.critic_optimizer)
        ]

        for iteration in range(1, num_updates + 1):
            buf = self.collect_rollout(envs)
            stats = self.update(buf)
            for scheduler in schedulers:
                scheduler.step()

            # Best checkpoint on the moving average of finished episodes
            if len(self.episode_returns) >= c.moving_avg_window:
                moving_avg = np.mean(self.episode_returns[-c.moving_avg_window:])
                if moving_avg >= self.best_moving_avg:
                    self.best_moving_avg = moving_avg
                    self.best_actor_state = copy.deepcopy(self.actor.state_dict())
                    self.best_episode = len(self.episode_returns)

            if iteration % c.print_every == 0:
                recent = self.episode_returns[-c.moving_avg_window:]
                mean_return = np.mean(recent) if recent else float("nan")
                print(
                    f"Update {iteration}/{num_updates} | steps {iteration * batch_size} | "
                    f"episodes {len(self.episode_returns)} | return {mean_return:.1f} | "
                    f"KL {stats['approx_kl']:.4f} | clip {stats['clip_frac']:.2f} | "
                    f"entropy {stats['entropy']:.3f}"
                )

        return self.episode_returns


    def evaluate(self, env: gym.Env, nb_episodes: int, deterministic: bool = False) -> list[float]:
        rewards = []
        for episode in range(nb_episodes):
            obs, info = env.reset()
            episode_reward = 0.0
            while True:
                action = self.get_action(obs, deterministic=deterministic)
                obs, reward, terminated, truncated, info = env.step(action.item())
                episode_reward += reward
                if terminated or truncated:
                    break
            rewards.append(episode_reward)
        return rewards


#################################################################################
##################################### PLOTS #####################################
#################################################################################


def create_run_dir(base_dir: Path) -> tuple[Path, int]:
    """Create the next free run folder (run_0001, run_0002, ...) and return it with its number."""
    base_dir.mkdir(parents=True, exist_ok=True)

    index = 1
    while True:
        run_dir = base_dir / f"run_{index:04d}"
        try:
            run_dir.mkdir()
            return run_dir, index
        except FileExistsError:
            index += 1


def describe_network(network: nn.Module) -> str:
    layers = [
        layer for layer in network.modules()
        if isinstance(layer, nn.Linear)
    ]

    if not layers:
        return "No linear layers"

    sizes = [layers[0].in_features]
    sizes.extend(layer.out_features for layer in layers)

    architecture = " → ".join(map(str, sizes))
    hidden_layers = len(layers) - 1

    activations = sorted({
        type(module).__name__ for module in network.modules()
        if not isinstance(module, (nn.Linear, nn.Sequential))
    })
    activation_text = f", {'/'.join(activations)}" if activations else ""

    return f"{architecture} ({hidden_layers} hidden layers{activation_text})"


def describe_config(config: Config, agent: PPOAgent) -> list[str]:
    """Text lines describing the run, read from the actual config, networks and env.

    Every Config field is listed automatically, so a new hyperparameter
    appears on the plots as soon as it is added to Config.
    """
    optimizer = agent.actor_optimizer
    settings = optimizer.defaults

    lines = [
        "CONFIGURATION",
        f"Algorithm: {ALGORITHM}",
        "",
    ]
    lines += [f"{field.name}: {getattr(config, field.name)}" for field in fields(config)]
    lines += [
        "",
        f"Actor: {describe_network(agent.actor)}",
        f"Critic: {describe_network(agent.critic)}",
        f"Optimizer: {type(optimizer).__name__}",
        f"  betas: {settings.get('betas')}",
        f"  eps: {settings.get('eps'):g}",
        f"  weight decay: {settings.get('weight_decay'):g}",
        f"Episode limit: {gym.spec(config.env_id).max_episode_steps} steps",
    ]
    return lines

def plot_evaluation_rewards(
    eval_rewards: dict[str, list[float]],
    config: Config,
    agent: PPOAgent,
    output_path: Path,
) -> Path:
    """Save and display evaluation returns for each action-selection mode."""
    if not eval_rewards or not all(eval_rewards.values()):
        raise ValueError("eval_rewards must contain at least one non-empty list")

    text = "\n".join(describe_config(config, agent) + [
        "",
        "EVALUATION",
        f"Modes: {', '.join(eval_rewards)}",
        f"Episodes per mode: {len(next(iter(eval_rewards.values())))}",
    ])

    fig, ax = plt.subplots(figsize=(13, 7))

    try:
        fig.subplots_adjust(right=0.63)

        colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
        max_episodes = 0

        for (mode, rewards), color in zip(eval_rewards.items(), colors):
            episodes = np.arange(1, len(rewards) + 1)
            mean_reward = np.mean(rewards)
            max_episodes = max(max_episodes, len(rewards))

            ax.plot(
                episodes,
                rewards,
                marker="o",
                color=color,
                label=f"{mode} — mean {mean_reward:.1f}",
            )
            ax.axhline(
                mean_reward,
                color=color,
                linestyle="--",
                alpha=0.6,
            )

        ax.set(
            title=f"{config.env_id} — evaluation of the best checkpoint",
            xlabel="Evaluation episode",
            ylabel="Total reward",
            ylim=(0, 525),
        )
        if max_episodes <= 25:
            ax.set_xticks(np.arange(1, max_episodes + 1))
        ax.grid(alpha=0.3)
        ax.legend(loc="lower left")

        fig.text(
            0.67, 0.94,
            text,
            va="top",
            ha="left",
            fontsize=9,
            fontfamily="monospace",
            linespacing=1.4,
        )

        fig.savefig(output_path, dpi=200, bbox_inches="tight")
        print(f"Plot saved to: {output_path}")
        plt.show()
    finally:
        plt.close(fig)

    return output_path


def moving_average(values: list[float], window: int) -> np.ndarray:
    return np.convolve(np.asarray(values, dtype=float), np.ones(window) / window, mode="valid")


def plot_training_rewards(
    episode_returns: list[float],
    config: Config,
    agent: PPOAgent,
    output_path: Path,
    window: int,
) -> Path:
    """Save and display the return of every training episode, with a moving average."""
    if not episode_returns:
        raise ValueError("episode_returns must not be empty")

    returns = np.asarray(episode_returns, dtype=float)
    episodes = np.arange(1, len(returns) + 1)

    text = "\n".join(describe_config(config, agent) + [
        "",
        "RESULTS",
        f"Episodes played: {len(returns)}",
        f"Last {min(window, len(returns))} ep. mean: {returns[-window:].mean():.1f}",
        f"Best episode: {returns.max():.0f}",
    ])

    fig, ax = plt.subplots(figsize=(13, 7))

    try:
        fig.subplots_adjust(right=0.63)

        ax.plot(episodes, returns, color="tab:blue", alpha=0.35, linewidth=1, label="Episode return")

        if len(returns) >= window:
            ax.plot(
                episodes[window - 1:],
                moving_average(returns, window),
                color="tab:blue",
                linewidth=2,
                label=f"Moving average ({window} episodes)",
            )

        ax.axhline(500, color="tab:green", linestyle=":", label="Max return (500)")

        ax.set(
            title=f"{config.env_id} — training episode returns",
            xlabel="Training episode",
            ylabel="Total reward",
            ylim=(0, 525),
            xlim=(1, len(returns)),
        )
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left")

        fig.text(0.67, 0.94, text, va="top", ha="left",
                 fontsize=9, fontfamily="monospace", linespacing=1.4)

        fig.savefig(output_path, dpi=200, bbox_inches="tight")
        print(f"Plot saved to: {output_path}")
        plt.show()
    finally:
        plt.close(fig)

    return output_path


#################################################################################
##################################### MAIN ######################################
#################################################################################

def main():
    config = Config()

    envs = [gym.make(config.env_id) for _ in range(config.num_envs)]
    try:
        agent = PPOAgent(
            config,
            obs_dim=envs[0].observation_space.shape[0],
            n_actions=envs[0].action_space.n,
        )
        training_returns = agent.train(envs)
    finally:
        for env in envs:
            env.close()

    if agent.best_actor_state is not None:
        agent.actor.load_state_dict(agent.best_actor_state)
        print(
            f"Loaded best actor at episode {agent.best_episode} "
            f"(moving average {agent.best_moving_avg:.1f})"
        )

    # Evaluation without rendering (fast), in both action-selection modes
    eval_env = gym.make(config.env_id)
    try:
        eval_rewards = {
            "Stochastic (sampling)": agent.evaluate(
                eval_env, nb_episodes=config.evaluation_episodes, deterministic=False,
            ),
            "Deterministic (argmax)": agent.evaluate(
                eval_env, nb_episodes=config.evaluation_episodes, deterministic=True,
            ),
        }
    finally:
        eval_env.close()

    for mode, rewards in eval_rewards.items():
        print(f"{mode}: mean evaluation return {np.mean(rewards):.1f}")

    # A few rendered episodes, just to watch the policy
    if config.render_episodes > 0:
        render_env = gym.make(config.env_id, render_mode="human")
        try:
            agent.evaluate(render_env, nb_episodes=config.render_episodes, deterministic=True)
        finally:
            render_env.close()

    # One folder per run; both plots share the run number
    plots_dir = Path(__file__).resolve().parent / "cartPole_PPO_plots"
    run_dir, run_index = create_run_dir(plots_dir)

    plot_training_rewards(
        training_returns, config, agent,
        output_path=run_dir / f"training_rewards_{run_index:04d}.png",
        window=config.moving_avg_window,
    )
    plot_evaluation_rewards(
        eval_rewards, config, agent,
        output_path=run_dir / f"evaluation_rewards_{run_index:04d}.png",
    )


if __name__ == "__main__":
    main()