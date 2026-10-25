# Run `pip install "gymnasium[classic-control]"` for this example.
import gymnasium as gym
import torch
from torch import nn
from torch.distributions import Categorical
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from dataclasses import dataclass

@dataclass
class Config:
    actor_learning_rate: float = 0.001
    critic_learning_rate: float = 0.005
    discount_factor: float = 0.95
    n_steps: int = 10
    training_episodes: int = 2000
    evaluation_episodes: int = 5

class CartPoleAgent:
    def __init__(self,
                #  env: gym.Env,
                 actor_learning_rate: float,
                 critic_learning_rate: float,
                 discount_factor: float = 0.95,
                 ):
        """Initialize a Reinforcement-Learning agent.

        Args:
            env: The training environment
            learning_rate: How quickly to update (0-1)
            discount_factor: How much to value future rewards (0-1)
        """

        self.actor_lr = actor_learning_rate
        self.critic_lr = critic_learning_rate
        self.discount_factor = discount_factor

        self.training_error = []

        self.actor = nn.Sequential(
            nn.Linear(4, 64),   # CartPole has 4 observation features
            nn.Tanh(),
            nn.Linear(64, 2),   # logits for left/right
        )
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=self.actor_lr)

        self.critic = nn.Sequential(
            nn.Linear(4,128),
            nn.Tanh(),
            nn.Linear(128, 1),  # predicted state value
        )
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=self.critic_lr)

    def get_action(self, obs: np.ndarray) -> torch.Tensor:
        state = torch.as_tensor(obs, dtype=torch.float32)
        with torch.no_grad():
            distribution = Categorical(logits=self.actor(state))
            action = distribution.sample()
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
    
    def update(self, env: gym.Env, nb_episodes: int, n_steps: int, states, next_states, actions, rewards, terminations):
        with torch.no_grad():
            next_values = self.critic(next_states).squeeze(-1)

        targets = compute_n_step_returns(rewards, next_values, terminations, self.discount_factor, n_steps)

        values = self.critic(states).squeeze(-1)
        advantages = targets - values

        distribution = Categorical(logits=self.actor(states))
        log_probs = distribution.log_prob(actions)

        actor_loss  = - (log_probs * advantages.detach()).mean()
        critic_loss = ((targets - values)**2).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()


    def train(self, env: gym.Env, nb_episodes: int, n_steps: int):
        for episode in range(nb_episodes):
                states, next_states, actions, rewards, terminations = self.collect_episode(env)

                self.update(env, nb_episodes, n_steps, states, next_states, actions, rewards, terminations)

                if (episode+1) % 50 == 0:
                    print(
                        f"Episode {episode + 1}: "
                        f"return={rewards.sum().item():.0f}"
                    )

    def evaluate(self, env: gym.Env, nb_episodes: int) -> list[float]:
        rewards = []

        for episode in range(nb_episodes):
            obs, info = env.reset()

            episode_reward = 0

            while True:
                action = self.get_action(obs)
                next_obs, reward, terminated, truncated, info = env.step(action.item())

                obs = next_obs
                episode_reward += reward

                if terminated or truncated:
                    break
            
            rewards.append(episode_reward)
        return rewards

def compute_n_step_returns(
    rewards: torch.Tensor,
    next_values: torch.Tensor,
    terminations: torch.Tensor,
    gamma: float,
    n_steps: int,
) -> torch.Tensor:
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

    return f"{architecture} ({hidden_layers} hidden layers)"

def plot_evaluation_rewards(
    episode_rewards: list[float],
    config: Config,
    agent: CartPoleAgent,
    output_dir: Path,
) -> Path:
    """Save and display evaluation returns with the training configuration."""
    if not episode_rewards:
        raise ValueError("episode_rewards must not be empty")

    episodes = np.arange(1, len(episode_rewards) + 1)
    mean_reward = np.mean(episode_rewards)
    optimizer_settings = agent.actor_optimizer.defaults

    hyperparameters = "\n".join([
        "TRAINING CONFIGURATION",
        "",
        "Algorithm: n-step actor–critic",
        f"Training episodes: {config.training_episodes}",
        "Update frequency: every episode",
        "Batch: all transitions in one episode",
        f"Actor learning rate: {agent.actor_lr:g}",
        f"Critic learning rate: {agent.critic_lr:g}",
        f"Discount factor: {agent.discount_factor:g}",
        "",
        f"Actor: {describe_network(agent.actor)}",
        f"Critic: {describe_network(agent.critic)}",
        "Hidden activation: Tanh",
        "Optimizer: Adam",
        f"Adam betas: {optimizer_settings['betas']}",
        f"Adam epsilon: {optimizer_settings['eps']:g}",
        f"Weight decay: {optimizer_settings['weight_decay']:g}",
        "",
        f"Return horizon: {config.n_steps} steps",
        "Entropy bonus: none",
        "Gradient clipping: none",
        "Advantage normalization: none",
        "Random seed: not fixed",
        "",
        "EVALUATION",
        f"Episodes: {len(episode_rewards)}",
        "Action selection: stochastic sampling",
        "Environment: CartPole-v1",
        "Episode limit: 500 steps",
    ])

    output_dir.mkdir(parents=True, exist_ok=True)

    index = 1
    while True:
        output_path = output_dir / f"evaluation_rewards_{index:04d}.png"
        if not output_path.exists():
            break
        index += 1

    fig, ax = plt.subplots(figsize=(13, 7))

    try:
        fig.subplots_adjust(right=0.63)

        ax.plot(
            episodes,
            episode_rewards,
            marker="o",
            label="Episode return",
        )
        ax.axhline(
            mean_reward,
            color="tab:orange",
            linestyle="--",
            label=f"Mean: {mean_reward:.1f}",
        )

        ax.set(
            title="CartPole — evaluation after training",
            xlabel="Evaluation episode",
            ylabel="Total reward",
            ylim=(0, 525),
        )
        ax.set_xticks(episodes)
        ax.grid(alpha=0.3)
        ax.legend()

        fig.text(
            0.67, 0.94,
            hyperparameters,
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



def main():
    config = Config()

    agent = CartPoleAgent(
        actor_learning_rate=config.actor_learning_rate,
        critic_learning_rate=config.critic_learning_rate,
        discount_factor=config.discount_factor,
    )

    env = gym.make("CartPole-v1")
    try:
        agent.train(
            env,
            nb_episodes=config.training_episodes,
            n_steps=config.n_steps,
        )
    finally:
        env.close()

    eval_env = gym.make("CartPole-v1", render_mode="human")
    try:
        episode_rewards = agent.evaluate(
            eval_env,
            nb_episodes=config.evaluation_episodes,
        )
    finally:
        eval_env.close()

    print(f"Mean evaluation return: {np.mean(episode_rewards):.1f}")

    output_dir = Path(__file__).resolve().parent / "cartPole_n-stepReturnA2C_plots"
    plot_evaluation_rewards(episode_rewards, config, agent, output_dir)



if __name__ == "__main__":
    main()