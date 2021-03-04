#!/usr/bin/env python

import argparse
from env import make_env
import random
import numpy as np
import cv2
import time
import suppressor

DIR_NAME = './data/rollout/'

WAYPOINT_REWARD_MULTIPLIER = 1.0
TIME_PENALTY = -0.1
SUPPRESS_STDOUT = True
SUPPRESS_STDERR = True

def main(args):

    env_name = args.env_name
    time_steps = args.time_steps
    total_episodes = args.total_episodes
    render = args.render
    action_refresh_rate = args.action_refresh_rate
    verbose = args.verbose
    verbose_stdout = args.verbose_stdout
    verbose_stderr = args.verbose_stderr

    SUPPRESS_STDOUT = True
    SUPPRESS_STDERR = True

    if verbose_stdout:
        SUPPRESS_STDOUT = False
    if verbose_stderr:
        SUPPRESS_STDERR = False

    
    episode_num = 0

    # with suppressor.suppress_output(suppress_stdout=SUPPRESS_STDOUT,suppress_stderr=SUPPRESS_STDERR):
    #     env = make_env(env_name, waypoint_reward_multi=WAYPOINT_REWARD_MULTIPLIER, timestep_reward=TIME_PENALTY, step_freq=60, render_mode='headless')

    while episode_num < total_episodes:

        with suppressor.suppress_output(suppress_stdout=SUPPRESS_STDOUT,suppress_stderr=SUPPRESS_STDERR):
            env = make_env(env_name, waypoint_reward_multi=WAYPOINT_REWARD_MULTIPLIER, timestep_reward=TIME_PENALTY, step_freq=60, render_mode='headless')

        episode_id = random.randint(0, 2**31 - 1)
        filename = DIR_NAME + str(episode_id) + ".npz"
        # input()
        with suppressor.suppress_output(suppress_stdout=SUPPRESS_STDOUT,suppress_stderr=SUPPRESS_STDERR):
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

            # input()
            
            if time_step_num % action_refresh_rate == 0:
                action = env.action_space.sample()
                # small push to explore environment
                if time_step_num < 60:
                    action[0] = 0.5
                    action[1] = 0
                if verbose:
                    print(f'action@timestep {time_step_num}: {action}')
            
            obs_sequence.append(observation)
            action_sequence.append(action)
            reward_sequence.append(reward)
            done_sequence.append(done)
            
            with suppressor.suppress_output(suppress_stdout=SUPPRESS_STDOUT,suppress_stderr=SUPPRESS_STDERR):
                observation, reward, done, debug_dict = env.step(action)
            if verbose:
                print(f'reward@timestep {time_step_num}: {reward}')

            if render:
                env.render()
            
            time_step_num += 1
        
        if render:
            cv2.destroyAllWindows()

        print("Episode {} finished after {} timesteps".format(episode_num, time_step_num))

        np.savez_compressed(filename, obs=obs_sequence, action=action_sequence,
                            reward=reward_sequence, done=done_sequence)
        
        episode_num += 1
        # env.reset()
        env.close()
        # print('Sleeping for 10s')
        # time.sleep(10)

def renderView(obs):
    cv2.namedWindow('observation', cv2.WINDOW_KEEPRATIO)
    obs = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR)
    cv2.imshow('observation', obs)
    cv2.resizeWindow('observation', 300, 300)
    cv2.waitKey(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=('Create new training data'))
    parser.add_argument('env_name', type=str, help='name of environment')
    parser.add_argument('--total_episodes', type=int, default=1000,
                        help='how many episodes to generate data?')
    parser.add_argument('--time_steps', type=int, default=100,
                        help='how many timesteps at start of episode?')
    parser.add_argument('--render', default=0, type=int,
                        help='render the env as data is generated')
    parser.add_argument('--action_refresh_rate', default=20, type=int,
                        help='how often to change the random action, in frames')
    parser.add_argument('--verbose', default=0, type=int,
                        help='get additional info into environment actions and rewards')
    parser.add_argument('--verbose_stderr', default=0, type=int,
                        help='Show stderr logs')
    parser.add_argument('--verbose_stdout', default=0, type=int,
                        help='show stdout logs')
    
    args = parser.parse_args()
    main(args)