import os
import gym
from expo_env import MultiAgentEnv
import time
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.results_plotter import load_results, ts2xy
# from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.callbacks import BaseCallback, CallbackList
import matplotlib.pyplot as plt


class PlottingCallback(BaseCallback):
    """
    Callback for plotting the performance in realtime.

    :param verbose: (int)
    """
    def __init__(self, verbose=1):
        super(PlottingCallback, self).__init__(verbose)
        self._plot = None

    def _on_step(self) -> bool:
        # get the monitor's data
        x, y = ts2xy(load_results(log_dir), 'timesteps')
        if self._plot is None: # make the plot
            plt.ion()
            fig = plt.figure(figsize=(6,3))
            ax = fig.add_subplot(111)
            line, = ax.plot(x, y)
            self._plot = (line, ax, fig)
            # plt.show()
        else: # update and rescale the plot
            self._plot[0].set_data(x, y)
            self._plot[-2].relim()
            self._plot[-2].set_xlim([self.locals["total_timesteps"] * -0.02, 
                                    self.locals["total_timesteps"] * 1.02])
            self._plot[-2].autoscale_view(True,True,True)
            self._plot[-1].canvas.draw()


class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq: (int)
    :param log_dir: (str) Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: (int)
    """
    def __init__(self, check_freq, log_dir, verbose=1):
        super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, 'best_model')
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            model_name = f"model_{int(self.n_calls / self.check_freq)}"
            self.save_path = os.path.join(log_dir, model_name)
            self.model.save(self.save_path)
        #   # Retrieve training reward
        #   x, y = ts2xy(load_results(self.log_dir), 'timesteps')
        #   if len(x) > 0:
        #       # Mean training reward over the last 100 episodes
        #       mean_reward = np.mean(y[-100:])
        #       if self.verbose > 0:
        #           pass
        #         #print("Num timesteps: {}".format(self.num_timesteps))
        #         #print("Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(self.best_mean_reward, mean_reward))

        #       # New best model, you could save the agent here
        #       if mean_reward > self.best_mean_reward:
        #           self.best_mean_reward = mean_reward
        #           # Example for saving best model
        #           if self.verbose > 0:
        #               pass
        #             #print("Saving new best model at {} timesteps".format(x[-1]))
        #             #print("Saving new best model to {}.zip".format(self.save_path))
                #   self.model.save(self.save_path)

        return True

# env = make_vec_env("hopping_leg-v0", n_envs=4)
log_dir = "./logs/26NovPPO_try1/"
os.makedirs(log_dir, exist_ok=True)

num_bots = 1  # Set number of bots
env = MultiAgentEnv(num_bots)
# env = make_vec_env("dp_gym-v0", n_envs=1, monitor_dir = log_dir)

# Callbacks
# save_callback = SaveOnBestTrainingRewardCallback(check_freq=240, log_dir=log_dir, verbose=1)
# plot_callback = PlottingCallback(log_dir=log_dir, verbose=1)
# plot_callback = PlottingCallback(verbose=1)
# callback = CallbackList([save_callback, plot_callback])

callback = SaveOnBestTrainingRewardCallback(check_freq=240, log_dir=log_dir, verbose=1)

# model = PPO("MlpPolicy", env, verbose=1, gamma = 1, use_sde = False)
model = PPO("MultiInputPolicy", env, verbose=1, gamma = 0.5, use_sde = False, n_steps=10,device="cuda")
model.learn(total_timesteps=int(24000), callback = callback)
model.save(log_dir + "/ppo_expo")

# env = gym.make("hopping_leg-v0", render = False, rail = False)
# obs = env.reset()