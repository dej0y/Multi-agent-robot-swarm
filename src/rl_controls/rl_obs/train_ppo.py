import rospy
from gazebo_env import GazeboEnv
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback

def main():
    # Create the environment
    env = GazeboEnv()

    # Define the model
    model = PPO('MlpPolicy', env, verbose=1)

    # Callbacks for saving best model and checkpoints
    eval_callback = EvalCallback(env, best_model_save_path='./logs/best_model',
                                 log_path='./logs/', eval_freq=1000,
                                 deterministic=True, render=False)
    checkpoint_callback = CheckpointCallback(save_freq=1000, save_path='./logs/',
                                             name_prefix='ppo_checkpoint')

    # Train the agent
    model.learn(total_timesteps=100000000)
    # model.learn(total_timesteps=500 * 200, callback=[eval_callback, checkpoint_callback])

    # Save the final model
    model.save('ppo_obstacle_avoidance')

if __name__ == '__main__':
    main()
