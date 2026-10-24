# Run `pip install "gymnasium[classic-control]"` for this example.
import gymnasium as gym
import torch
from torch import nn
from torch.distributions import Categorical
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

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

        # self.env = env

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
            nn.Linear(4, 64),
            nn.Tanh(),
            nn.Linear(64, 1),  # predicted state value
        )
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=self.critic_lr)

    def get_action(self, obs: np.ndarray) -> torch.Tensor:
        state = torch.as_tensor(obs, dtype=torch.float32)
        with torch.no_grad():
            distribution = Categorical(logits=self.actor(state))
            action = distribution.sample()
        return action

    def update(self, env: gym.Env, nb_episodes: int, n_steps: int):

        for episode in range(nb_episodes):
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

            targets = torch.zeros_like(rewards)

            with torch.no_grad():
                next_values = self.critic(next_states).squeeze(-1)
                for t in range(len(rewards)):
                    m = min(n_steps, len(rewards)-t)
                    for i in range(m):
                        targets[t] += rewards[t+i] * self.discount_factor**i
                    targets[t] += self.discount_factor**m * (~terminations[t+m-1]).float() * next_values[t+m-1]

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

            if (episode+1) % 50 == 0:
                print(
                    f"Episode {episode + 1}: "
                    f"return={rewards.sum().item():.0f}"
                )

    def evaluate(self, env: gym.Env, num_episodes: int) -> list[float]:
        rewards = []

        for episode in range(num_episodes):
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


            

# Create our training environment - a cart with a pole that needs balancing
agent = CartPoleAgent(actor_learning_rate=0.001, critic_learning_rate=0.005, discount_factor=0.95)

training_episodes = 2000
n_steps = 10

env = gym.make("CartPole-v1") #render_mode="human"
agent.update(env, training_episodes, n_steps)
env.close()

eval_env = gym.make("CartPole-v1", render_mode="human")
episode_rewards = agent.evaluate(eval_env, num_episodes=5)
eval_env.close()

print(f"Mean evaluation return: {np.mean(episode_rewards):.1f}")



# Plots
episodes = np.arange(1, len(episode_rewards) + 1)

hyperparameters = "\n".join([
    "TRAINING CONFIGURATION",
    "",
    "Algorithm: n-step actor–critic",
    f"Training episodes: {training_episodes}",
    "Update frequency: every episode",
    "Batch: all transitions in one episode",
    f"Actor learning rate: {agent.actor_lr:g}",
    f"Critic learning rate: {agent.critic_lr:g}",
    f"Discount factor: {agent.discount_factor:g}",
    "",
    "Actor: 4 → 64 → 2",
    "Critic: 4 → 64 → 1",
    "Hidden activation: Tanh",
    "Optimizer: Adam",
    f"Adam betas: {agent.actor_optimizer.defaults['betas']}",
    f"Adam epsilon: {agent.actor_optimizer.defaults['eps']:g}",
    f"Weight decay: {agent.actor_optimizer.defaults['weight_decay']:g}",
    "",
    f"Return horizon: {n_steps} steps",
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

fig, ax = plt.subplots(figsize=(13, 7))
fig.subplots_adjust(right=0.63)

ax.plot(
    episodes,
    episode_rewards,
    marker="o",
    label="Episode return",
)
ax.axhline(
    np.mean(episode_rewards),
    color="tab:orange",
    linestyle="--",
    label=f"Mean: {np.mean(episode_rewards):.1f}",
)

ax.set(
    title="CartPole — evaluation after training",
    xlabel="Evaluation episode",
    ylabel="Total reward",
)
ax.set_xticks(episodes)
ax.set_ylim(0, 525)
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

# Create the output folder next to this script.
output_dir = Path(__file__).resolve().parent / "cartPole_n-stepReturnA2C_plots"

index = 1
while True:
    output_path = output_dir / f"evaluation_rewards_{index:04d}.png"
    if not output_path.exists():
        break
    index += 1

fig.savefig(output_path, dpi=200, bbox_inches="tight")
print(f"Plot saved to: {output_path}")

plt.show()
plt.close(fig)