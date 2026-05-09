from stable_baselines3 import PPO
from custom_ros_env import MultiRobotEnv  # Custom environment for merged map and exploration
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.vec_env import DummyVecEnv
import os

# env = MultiRobotEnv()
env = DummyVecEnv([lambda: MultiRobotEnv()])
env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)

# model = PPO("MlpPolicy", env, verbose=1)
model = PPO("MlpPolicy", env, verbose=1, learning_rate=3e-4, clip_range=0.2, vf_coef=0.5)

# model.learn(total_timesteps=100000000)

# Define the model
# model = PPO('MlpPolicy', env, verbose=1)

# Callbacks for saving best model and checkpoints
eval_callback = EvalCallback(env, best_model_save_path='./logs/best_model',
                                log_path='./logs/', eval_freq=1000,
                                deterministic=True, render=False)
checkpoint_callback = CheckpointCallback(save_freq=1000, save_path='./logs/',
                                            name_prefix='ppo_checkpoint')

# Train the agent
# model.learn(total_timesteps=100000000)
model.learn(total_timesteps=5000*2000, callback=[eval_callback, checkpoint_callback])

# Save the model
model.save("ppo_exploration_model")
