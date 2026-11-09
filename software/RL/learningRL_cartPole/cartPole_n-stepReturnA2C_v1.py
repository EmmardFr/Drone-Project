# Run `pip install "gymnasium[classic-control]"` for this example.
import gymnasium as gym
import torch
from torch import nn
from torch.distributions import Categorical
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from dataclasses import dataclass, fields
import copy
import random


ALGORITHM = "n-step actor-critic, 1 update per episode (all its transitions)"


@dataclass
class Config:
    env_id: str = "CartPole-v1"
    actor_learning_rate: float = 0.001
    critic_learning_rate: float = 0.005
    discount_factor: float = 0.95
    entropy_coef: float = 0.001
    n_steps: int = 10
    training_episodes: int = 200
    evaluation_episodes: int = 20   # per action-selection mode, without rendering
    render_episodes: int = 1        # watched episodes after evaluation (0 = none)
    max_grad_norm: float = 0.5
    moving_avg_window: int = 50
    print_every: int = 50
    seeds: tuple[int, ...] = (0, 1, 2)


def set_seed(seed: int) -> None:
    """Make network init, PyTorch sampling and NumPy reproducible."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)

class CartPoleAgent:
    def __init__(self,
                 actor_learning_rate: float,
                 critic_learning_rate: float,
                 entropy_coef: float,
                 max_grad_norm: float,
                 discount_factor: float = 0.95,
                 ):
        """Initialize an n-step actor-critic agent.

        Args:
            actor_learning_rate: Adam learning rate of the policy network
            critic_learning_rate: Adam learning rate of the value network
            entropy_coef: Weight of the entropy bonus in the actor loss
            discount_factor: How much to value future rewards (0-1)
        """

        self.actor_lr = actor_learning_rate
        self.critic_lr = critic_learning_rate
        self.discount_factor = discount_factor
        self.entropy_coef = entropy_coef
        self.max_grad_norm = max_grad_norm

        self.actor = nn.Sequential(
            nn.Linear(4, 64),   # CartPole has 4 observation features
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, 2),   # logits for left/right
        )
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=self.actor_lr)

        self.critic = nn.Sequential(
            nn.Linear(4,64),
            nn.Tanh(),
            # nn.Linear(128,128),
            # nn.Tanh(),
            nn.Linear(64, 1),  # predicted state value
        )
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=self.critic_lr)

    def get_action(self, obs: np.ndarray, deterministic: bool = False) -> torch.Tensor:
        """Sample an action (training, exploration) or take the most probable one (deterministic)."""
        state = torch.as_tensor(obs, dtype=torch.float32)
        with torch.no_grad():
            logits = self.actor(state)
            if deterministic:
                action = logits.argmax()
            else:
                action = Categorical(logits=logits).sample()
        return action

    def collect_episode(self, env: gym.Env, seed: int | None = None):
        obs, info = env.reset(seed=seed)

        states       = []
        next_states  = []
        actions      = []
        rewards      = []
        terminations = []

        while True:
            action = self.get_action(obs)
            next_obs, reward, terminated, truncated, info = env.step(action.item())

            actions.append(action)
            states.append(obs)
            next_states.append(next_obs)
            rewards.append(reward)
            terminations.append(terminated)
            obs = next_obs

            if terminated or truncated:
                break

        states       = torch.as_tensor(np.array(states), dtype=torch.float32)
        next_states  = torch.as_tensor(np.array(next_states), dtype=torch.float32)
        actions      = torch.stack(actions).to(dtype=torch.int64)
        rewards      = torch.as_tensor(np.array(rewards), dtype=torch.float32)
        terminations = torch.as_tensor(terminations, dtype=torch.bool)

        return states, next_states, actions, rewards, terminations

    def update(self, n_steps: int, states, next_states, actions, rewards, terminations):
        with torch.no_grad():
            next_values = self.critic(next_states).squeeze(-1)

        targets = compute_n_step_returns(rewards, next_values, terminations, self.discount_factor, n_steps)

        values = self.critic(states).squeeze(-1)
        advantages = targets - values

        distribution = Categorical(logits=self.actor(states))
        log_probs = distribution.log_prob(actions)

        entropy = distribution.entropy().mean()
        actor_loss  = - (log_probs * advantages.detach()).mean() - self.entropy_coef * entropy
        critic_loss = ((targets - values)**2).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=self.max_grad_norm)
        self.actor_optimizer.step()

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=self.max_grad_norm)
        self.critic_optimizer.step()


    def train(self, env: gym.Env, nb_episodes: int, n_steps: int, window: int, print_every: int, seed: int | None = None) -> list[float]:
        episode_returns = []

        self.best_moving_avg = -float("inf")
        self.best_actor_state = None
        self.best_episode = None

        for episode in range(nb_episodes):
            states, next_states, actions, rewards, terminations = self.collect_episode(env, seed=seed if episode == 0 else None)

            self.update(n_steps, states, next_states, actions, rewards, terminations)

            episode_return = rewards.sum().item()
            episode_returns.append(episode_return)

            if len(episode_returns) >= window:
                moving_avg = np.mean(episode_returns[-window:])
                if moving_avg >= self.best_moving_avg:
                    self.best_moving_avg = moving_avg
                    self.best_actor_state = copy.deepcopy(self.actor.state_dict())
                    self.best_episode = episode + 1

            if (episode + 1) % print_every == 0:
                print(
                    f"Episode {episode + 1}: "
                    f"return={episode_return:.0f}"
                )

        return episode_returns

    def evaluate(self, env: gym.Env, nb_episodes: int, deterministic: bool = False, seed: int | None = None) -> list[float]:
        rewards = []

        for episode in range(nb_episodes):
            obs, info = env.reset(seed=seed if episode == 0 else None)

            episode_reward = 0

            while True:
                action = self.get_action(obs, deterministic=deterministic)
                next_obs, reward, terminated, truncated, info = env.step(action.item())

                obs = next_obs
                episode_reward += reward

                if terminated or truncated:
                    break

            rewards.append(episode_reward)
        return rewards

def compute_n_step_returns(rewards: torch.Tensor, next_values: torch.Tensor, terminations: torch.Tensor, gamma: float, n_steps: int) -> torch.Tensor:
    """Compute targets for one episode or a fragment ending at its boundary.

    All input tensors have shape [L].
    No episode reset may occur inside the sequence.
    Vectorized over t: n_steps operations on whole tensors instead of L × n_steps scalar ones.
    """
    if n_steps < 1:
        raise ValueError("n_steps must be at least 1")

    with torch.no_grad():
        length = len(rewards)
        targets = torch.zeros_like(rewards)

        # Sum the rewards actually observed: for each k, add gamma^k * r[t+k] to every t at once.
        # Slicing stops automatically at the end of the episode (that's the min(n_steps, L - t)).
        for k in range(min(n_steps, length)):
            targets[:length - k] += gamma**k * rewards[k:]

        # Bootstrap from the critic at the last transition of each window, unless it terminated.
        t = torch.arange(length)
        last = torch.clamp(t + n_steps, max=length) - 1      # index of the transition leading to the bootstrap state
        m = last - t + 1                                     # number of rewards actually summed
        not_terminal = (~terminations[last]).float()
        targets += (gamma ** m.float()) * next_values[last] * not_terminal

    return targets


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


def describe_config(config: Config, agent: CartPoleAgent) -> list[str]:
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
    agent: CartPoleAgent,
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
    returns_per_seed: dict[int, list[float]],
    config: Config,
    agent: CartPoleAgent,
    output_path: Path,
    window: int,
) -> Path:
    """Moving average of the training return for each seed, with mean ± std across seeds."""
    if not returns_per_seed:
        raise ValueError("returns_per_seed must not be empty")

    n_episodes = len(next(iter(returns_per_seed.values())))
    if n_episodes < window:
        raise ValueError(f"Need at least {window} episodes for the moving average")

    curves = np.array([moving_average(r, window) for r in returns_per_seed.values()])
    mean = curves.mean(axis=0)
    std = curves.std(axis=0)
    x = np.arange(window, n_episodes + 1)   # each average covers the episodes before it

    best = int(mean.argmax())
    text = "\n".join(describe_config(config, agent) + [
        "",
        f"RESULTS ({len(returns_per_seed)} seeds, moving average)",
        f"Final: {mean[-1]:.1f} ± {std[-1]:.1f}",
        f"Best mean: {mean[best]:.1f} ± {std[best]:.1f} (ep. {x[best]})",
    ])

    fig, ax = plt.subplots(figsize=(13, 7))

    try:
        fig.subplots_adjust(right=0.63)

        for seed, curve in zip(returns_per_seed, curves):
            ax.plot(x, curve, linewidth=1, alpha=0.5, label=f"Seed {seed}")

        ax.plot(x, mean, color="black", linewidth=2.5, label=f"Mean of {len(curves)} seeds")
        ax.fill_between(x, mean - std, mean + std, color="gray", alpha=0.25, label="± 1 std")

        ax.axhline(500, color="tab:green", linestyle=":", label="Max return (500)")

        ax.set(
            title=f"{config.env_id} — training return ({window}-episode moving average)",
            xlabel="Training episode",
            ylabel="Total reward",
            ylim=(0, 525),
            xlim=(1, n_episodes),
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

def train_one_seed(config: Config, seed: int) -> tuple[CartPoleAgent, list[float]]:
    set_seed(seed)   # before creating the agent: network weights depend on it

    agent = CartPoleAgent(
        actor_learning_rate=config.actor_learning_rate,
        critic_learning_rate=config.critic_learning_rate,
        discount_factor=config.discount_factor,
        entropy_coef=config.entropy_coef,
        max_grad_norm=config.max_grad_norm,
    )

    env = gym.make(config.env_id)
    try:
        training_returns = agent.train(
            env,
            nb_episodes=config.training_episodes,
            n_steps=config.n_steps,
            window=config.moving_avg_window,
            print_every=config.print_every,
            seed=seed,
        )
    finally:
        env.close()

    if agent.best_actor_state is not None:
        agent.actor.load_state_dict(agent.best_actor_state)
        print(
            f"Loaded best actor from episode {agent.best_episode} "
            f"(moving average {agent.best_moving_avg:.1f})"
        )

    return agent, training_returns


def main():
    config = Config()

    modes = {"Stochastic (sampling)": False, "Deterministic (argmax)": True}

    agents: dict[int, CartPoleAgent] = {}
    training_returns: dict[int, list[float]] = {}
    eval_rewards: dict[str, list[float]] = {mode: [] for mode in modes}
    eval_means: dict[str, list[float]] = {mode: [] for mode in modes}

    for seed in config.seeds:
        print(f"\n===== Seed {seed} =====")
        agent, returns = train_one_seed(config, seed)
        agents[seed] = agent
        training_returns[seed] = returns

        # Evaluation without rendering; a different seed than training
        eval_env = gym.make(config.env_id)
        try:
            for mode, deterministic in modes.items():
                rewards = agent.evaluate(
                    eval_env,
                    nb_episodes=config.evaluation_episodes,
                    deterministic=deterministic,
                    seed=seed + 10_000,
                )
                eval_rewards[mode].extend(rewards)
                eval_means[mode].append(np.mean(rewards))
                print(f"{mode}: mean evaluation return {np.mean(rewards):.1f}")
        finally:
            eval_env.close()

    print(f"\n===== Summary over {len(config.seeds)} seeds =====")
    for mode, means in eval_means.items():
        print(f"{mode}: {np.mean(means):.1f} ± {np.std(means):.1f}")

    # Watch the seed whose training reached the best moving average
    if config.render_episodes > 0:
        best_seed = max(agents, key=lambda s: agents[s].best_moving_avg)
        print(f"Rendering seed {best_seed}")
        render_env = gym.make(config.env_id, render_mode="human")
        try:
            agents[best_seed].evaluate(
                render_env, nb_episodes=config.render_episodes, deterministic=True,
            )
        finally:
            render_env.close()

    # One folder per run; both plots share the run number
    plots_dir = Path(__file__).resolve().parent / "cartPole_n-stepReturnA2C_plots"
    run_dir, run_index = create_run_dir(plots_dir)

    any_agent = next(iter(agents.values()))   # only used for the network description
    plot_training_rewards(
        training_returns, config, any_agent,
        output_path=run_dir / f"training_rewards_{run_index:04d}.png",
        window=config.moving_avg_window,
    )
    plot_evaluation_rewards(
        eval_rewards, config, any_agent,
        output_path=run_dir / f"evaluation_rewards_{run_index:04d}.png",
    )


if __name__ == "__main__":
    main()