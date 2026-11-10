# does not work

import logging
import gymnasium as gym
from gymnasium.wrappers import RecordEpisodeStatistics, RecordVideo
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
from tqdm import tqdm  # Progress bar




class CartPoleAgent:
    def __init__(self,
                 env: gym.Env,
                 learnning_rate: float,
                 initial_epsilon: float,
                 epsilon_decay: float,
                 final_epsilon: float,
                 discount_factor: float = 0.95,
                 ):
        """Initialize a Q-Learning agent.

        Args:
            env: The training environment
            learning_rate: How quickly to update Q-values (0-1)
            initial_epsilon: Starting exploration rate (usually 1.0)
            epsilon_decay: How much to reduce epsilon each episode
            final_epsilon: Minimum exploration rate (usually 0.1)
            discount_factor: How much to value future rewards (0-1)
        """

        self.env = env

        self.qvalues = defaultdict(lambda: np.zeros(env.action_space.n))

        self.lr = learnning_rate
        self.discount_factor = discount_factor

        self.epsilon = initial_epsilon
        self.epsilon_decay = epsilon_decay
        self.final_epsilon = final_epsilon

        self.training_error = []

    def get_action(self, obs: tuple[int, int, bool]) -> int:
        """Choose an action using epsilon-greedy strategy.

        Returns:
            action: 0 (stand) or 1 (hit)
        """


        if np.random.random() < self.epsilon:
            return self.env.action_space.sample()

        return int(np.argmax(tuple(self.qvalues[obs])))

    def update(
            self,
            obs: tuple[int, int, bool],
            action: int,
            reward: float,
            terminated: bool,
            next_obs: tuple[int, int, bool],
            ):
        """Update Q-value based on experience.

        This is the heart of Q-learning: learn from (state, action, reward, next_state)
        """

        future_q_value = (not terminated) * np.max(tuple(self.qvalues[next_obs]))

        target = reward + self.discount_factor * future_q_value

        temporal_difference = target - self.qvalues[obs][action]

        self.qvalues[obs][action] += self.lr * temporal_difference

        self.training_error.append(temporal_difference)

    def decay_epsilon(self):
        self.epsilon = max(self.epsilon - self.epsilon_decay, self.final_epsilon)



# Training configuration
training_period = 250           # Record video every 250 episodes
num_training_episodes = 10_000  # Total training episodes
env_name = "CartPole-v1"

# Set up logging for episode statistics
logging.basicConfig(level=logging.INFO, format='%(message)s')

# Create environment with periodic video recording
env = gym.make(env_name, render_mode="rgb_array")

# Record videos periodically (every 250 episodes)
env = RecordVideo(
    env,
    video_folder="cartpole-training",
    name_prefix="training",
    episode_trigger=lambda x: x % training_period == 0  # Only record every 250th episode
)

# Track statistics for every episode (lightweight)
env = RecordEpisodeStatistics(env)

agent = CartPoleAgent(
    env=env,
    learnning_rate=0.1,
    initial_epsilon=1.0,
    epsilon_decay=0.001,
    final_epsilon=0.1,
    discount_factor=0.95
)

print(f"Starting training for {num_training_episodes} episodes")
print(f"Videos will be recorded every {training_period} episodes")
print(f"Videos saved to: cartpole-training/")

for episode_num in range(num_training_episodes):
    obs, info = env.reset()
    episode_over = False

    while not episode_over:
        # Replace with your actual training agent
        # action = env.action_space.sample()  # Random policy for demonstration
        action = agent.get_action(obs)
        next_obs, reward, terminated, truncated, info = env.step(action)
        agent.update(obs, action, reward, terminated, next_obs)
        obs = next_obs
        episode_over = terminated or truncated

    agent.decay_epsilon()

    # Log episode statistics (available in info after episode ends)
    if "episode" in info:
        episode_data = info["episode"]
        logging.info(f"Episode {episode_num}: "
                    f"reward={episode_data['r']:.1f}, "
                    f"length={episode_data['l']}, "
                    f"time={episode_data['t']:.2f}s")

        # Additional analysis for milestone episodes
        if episode_num % 1000 == 0:
            # Look at recent performance (last 100 episodes)
            recent_rewards = list(env.return_queue)[-100:]
            if recent_rewards:
                avg_recent = sum(recent_rewards) / len(recent_rewards)
                print(f"  -> Average reward over last 100 episodes: {avg_recent:.1f}")

env.close()


# Plot learning progress
episodes = range(len(env.return_queue))
rewards = list(env.return_queue)

plt.figure(figsize=(10, 6))
plt.plot(episodes, rewards, alpha=0.3, label='Episode Rewards')

# Add moving average for clearer trend
window = 100
if len(rewards) > window:
    moving_avg = [sum(rewards[i:i+window])/window
                  for i in range(len(rewards)-window+1)]
    plt.plot(range(window-1, len(rewards)), moving_avg,
             label=f'{window}-Episode Moving Average', linewidth=2)

plt.xlabel('Episode')
plt.ylabel('Reward')
plt.title('Learning Progress')
plt.legend()
plt.grid(True, alpha=0.3)
plt.show()