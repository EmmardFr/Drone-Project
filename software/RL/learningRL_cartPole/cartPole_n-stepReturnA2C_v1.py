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


ALGORITHM = "n-step actor-critic, 1 update per episode (all its transitions)"


@dataclass
class Config:
    env_id: str = "CartPole-v1"
    actor_learning_rate: float = 0.001
    critic_learning_rate: float = 0.005
    discount_factor: float = 0.95
    entropy_coef: float = 0.001
    n_steps: int = 10
    training_episodes: int = 1000
    evaluation_episodes: int = 20   # per action-selection mode, without rendering
    render_episodes: int = 1        # watched episodes after evaluation (0 = none)
    max_grad_norm: float = 0.5
    moving_avg_window: int = 50
    print_every: int = 50


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

    def collect_episode(self, env: gym.Env):
        obs, info = env.reset()

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


    def train(self, env: gym.Env, nb_episodes: int, n_steps: int, window: int, print_every: int) -> list[float]:
        episode_returns = []

        self.best_moving_avg = -float("inf")
        self.best_actor_state = None
        self.best_episode = None

        for episode in range(nb_episodes):
            states, next_states, actions, rewards, terminations = self.collect_episode(env)

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

    def evaluate(self, env: gym.Env, nb_episodes: int, deterministic: bool = False) -> list[float]:
        rewards = []

        for episode in range(nb_episodes):
            obs, info = env.reset()

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
    """
    if n_steps < 1:
        raise ValueError("n_steps must be at least 1")

    with torch.no_grad():
        targets = torch.zeros_like(rewards)
        length = len(rewards)

        for t in range(length):
            m = min(n_steps, length - t)

            # Sum the rewards actually observed.
            for k in range(m):
                targets[t] += gamma**k * rewards[t + k]

            # Index of the transition leading to the bootstrap state.
            last = t + m - 1

            if not terminations[last].item():
                targets[t] += gamma**m * next_values[last]

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


def plot_training_rewards(
    episode_returns: list[float],
    config: Config,
    agent: CartPoleAgent,
    output_path: Path,
    window: int,
) -> Path:
    """Save and display the return of every training episode, with a moving average."""
    if not episode_returns:
        raise ValueError("episode_returns must not be empty")

    returns = np.asarray(episode_returns, dtype=float)
    episodes = np.arange(1, len(returns) + 1)

    summary = "\n".join([
        "TRAINING",
        f"Episodes: {config.training_episodes}",
        f"Actor lr: {agent.actor_lr:g}",
        f"Critic lr: {agent.critic_lr:g}",
        f"Discount factor: {agent.discount_factor:g}",
        f"Return horizon: {config.n_steps} steps",
        f"Entropy coefficient: {agent.entropy_coef:g}",
        f"Actor: {describe_network(agent.actor)}",
        f"Critic: {describe_network(agent.critic)}",
        "",
        f"Last {min(window, len(returns))} ep. mean: {returns[-window:].mean():.1f}",
        f"Best episode: {returns.max():.0f}",
    ])

    fig, ax = plt.subplots(figsize=(13, 7))

    try:
        fig.subplots_adjust(right=0.70)

        ax.plot(
            episodes,
            returns,
            color="tab:blue",
            alpha=0.35,
            linewidth=1,
            label="Episode return",
        )

        if len(returns) >= window:
            moving_avg = np.convolve(returns, np.ones(window) / window, mode="valid")
            ax.plot(
                episodes[window - 1:],
                moving_avg,
                color="tab:blue",
                linewidth=2,
                label=f"Moving average ({window} episodes)",
            )

        ax.axhline(
            500,
            color="tab:green",
            linestyle=":",
            label="Max return (500)",
        )

        ax.set(
            title="CartPole — training episode returns",
            xlabel="Training episode",
            ylabel="Total reward",
            ylim=(0, 525),
            xlim=(1, len(returns)),
        )
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left")

        fig.text(
            0.73, 0.94,
            summary,
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


#################################################################################
##################################### MAIN ######################################
#################################################################################

def main():
    config = Config()

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
        )
    finally:
        env.close()

    if agent.best_actor_state is not None:
        agent.actor.load_state_dict(agent.best_actor_state)
        print(
            f"Loaded best actor from episode {agent.best_episode} "
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
    plots_dir = Path(__file__).resolve().parent / "cartPole_n-stepReturnA2C_plots"
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