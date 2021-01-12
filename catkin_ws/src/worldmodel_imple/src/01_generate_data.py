#!/usr/bin/env python

import argparse
from env import make_env
import random
import numpy as np
import cv2
import time

DIR_NAME = './data/rollout/'

WAYPOINT_REWARD_MULTIPLIER = 1.0
TIME_PENALTY = -0.1

def main(args):

    env_name = args.env_name
    time_steps = args.time_steps
    render = args.render
    action_refresh_rate = args.action_refresh_rate

    episode_num = 0

    episode_id = random.randint(0, 2**31 - 1)
    filename = DIR_NAME + str(episode_id) + ".npz"
    track_name = 'track'+str(random.randint(1,20))

    env = make_env('SingleRacecar', track_name=track_name, waypoint_reward_mult=WAYPOINT_REWARD_MULTIPLIER, time_reward=TIME_PENALTY, gazebo_gui=True, step_size=0.02)

    # if render:
    #     env._render()

    observation = env.reset()

    obs_sequence = []
    action_sequence = []
    reward_sequence = []
    done_sequence = []

    reward = -0.1
    done = False

    time_step_num = 0

    # input('Paused, waiting for keyboard input')

    while time_step_num < time_steps:
        
        if time_step_num % action_refresh_rate == 0:
            action = env.action_space.sample()
            print(action)
        
        obs_sequence.append(observation)
        action_sequence.append(action)
        reward_sequence.append(reward)
        done_sequence.append(done)
        
        observation, reward, done, debug_dict = env.step(action)
        print(reward)

        if render:
            renderView(observation)
        
        time_step_num += 1
    
    if render:
        cv2.destroyAllWindows()

    print("Episode {} finished after {} timesteps".format(episode_num, time_step_num))

    np.savez_compressed(filename, obs=obs_sequence, action=action_sequence,
                        reward=reward_sequence, done=done_sequence)
    
    env._close()

    env = None

    episode_num += 1
    # print('Sleeping for 10s')
    # time.sleep(10)

def renderView(obs):
    cv2.namedWindow('observation', cv2.WINDOW_KEEPRATIO)
    cv2.imshow('observation', obs)
    cv2.resizeWindow('observation', 300, 300)
    cv2.waitKey(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=('Create new training data'))
    parser.add_argument('env_name', type=str, help='name of environment')
    parser.add_argument('--time_steps', type=int, default=100,
                        help='how many timesteps at start of episode?')
    parser.add_argument('--render', default=0, type=int,
                        help='render the env as data is generated')
    parser.add_argument('--action_refresh_rate', default=20, type=int,
                        help='how often to change the random action, in frames')
    
    args = parser.parse_args()
    main(args)