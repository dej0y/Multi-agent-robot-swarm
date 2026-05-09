import os
import gym
import time
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.results_plotter import load_results, ts2xy

class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving the best model based on the training reward.
    
    :param check_freq: (int) Frequency (in steps) to check the reward.
    :param log_dir: (str) Path where the model will be saved.
    :param verbose: (int) Verbosity level.
    """
    def __init__(self, check_freq, log_dir, verbose=1):
        super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, 'best_model')
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        # Create folder for saving models
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            model_name = f"model_{int(self.n_calls / self.check_freq)}"
            self.save_path = os.path.join(self.log_dir, model_name)
            self.model.save(self.save_path)
            
            # Retrieve training reward stats
            x, y = ts2xy(load_results(self.log_dir), 'timesteps')
            if len(x) > 0:
                # Mean training reward over the last 100 episodes
                mean_reward = np.mean(y[-100:])
                if self.verbose > 0:
                    print(f"Timesteps: {x[-1]}, Mean reward: {mean_reward:.2f}")

                # Save the model if it's the best so far
                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    if self.verbose > 0:
                        print(f"Saving new best model with reward {mean_reward:.2f} at {x[-1]} timesteps")
                    self.model.save(self.save_path)
        return True

# Path for saving logs and models
log_dir = "logs"
os.makedirs(log_dir, exist_ok=True)

# Set up your environment (make sure 'MultiAgentEnv' is the correct environment)
env = gym.make('FrontierExploreEnv-v0')  # Adjust the name of your custom environment

# Define the callback
callback = SaveOnBestTrainingRewardCallback(check_freq=240, log_dir=log_dir, verbose=1)

# Initialize PPO model (with MultiInputPolicy for handling multiple input spaces)
model = PPO("MultiInputPolicy", env, verbose=1, gamma=0.5, use_sde=False, n_steps=10, device="cuda")

# Train the model for a total of 24,000 timesteps
model.learn(total_timesteps=24000, callback=callback)

# Save the trained model after the learning is complete
model.save(os.path.join(log_dir, "ppo_expo"))



# Load the results (training stats)
x, y = ts2xy(load_results(log_dir), 'timesteps')

# Plot the results
plt.plot(x, y)
plt.xlabel('Timesteps')
plt.ylabel('Mean Reward')
plt.title('Training Progress')
plt.show()
