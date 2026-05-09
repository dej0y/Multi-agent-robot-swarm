#! /usr/bin/env python
import tensorflow as tf
import keras.backend as K
import rospy
import multi_bot_voronoi_explore
import ddpg_network_per_replay_human
import time

sess = tf.Session()
K.set_session(sess)

num_trials = 10000
trial_len  = 500
train_indicator = 0

num_bots = 4

robot_name = [f'bot{i+1}' for i in range(num_bots)]

complete_num = 0
for i in range(num_trials):
    game_state= multi_bot_voronoi_explore.GameState(num_bots_online=num_bots)   # game_state has frame_step(action) function
    actor_critic = ddpg_network_per_replay_human.ActorCritic(game_state, sess)
    actor_critic.actor_model.load_weights("/home/shashwatgpatil/InterIIT_midprep_new/src/turtlebot_ddpg/scripts/original_ddpg/actormodel-300-500.h5")
    actor_critic.critic_model.load_weights("/home/shashwatgpatil/InterIIT_midprep_new/src/turtlebot_ddpg/scripts/original_ddpg/criticmodel-300-500.h5")
    # print("trial:" + str(i))
    start_time =time.time()
    # print("this is before game state reset ")
    current_state = game_state.reset()
    # print("current state print", current_state)
    total_reward = 0
    for j in range(trial_len):
        if game_state.done == True:
            game_state.done = False
            # game_state.shut_down()
            rospy.sleep(2)
            complete_num +=1
            break
        for k in range(num_bots):
            current_state[k] = current_state[k].reshape((1, game_state.observation_space.shape[0]))
            action = actor_critic.play(current_state[k])  # need to change the network input output, do I need to change the output to be [0, 2*pi]
            action = action.reshape((1, game_state.action_space.shape[0]))
            if game_state.laser_crashed_value[robot_name[k]] == True:
                game_state.game_step(robot_name[k],0.1, 0, 0)
            else:
                new_state = game_state.game_step(robot_name[k],0.1, action[0][1], action[0][0]) # we get reward and state here, then we need to calculate if it is crashed! for 'dones' value

            new_state = new_state.reshape((1, game_state.observation_space.shape[0]))
            current_state[k] = new_state
    end_time = time.time()
    # print("total:",end_time-start_time)