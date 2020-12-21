#!/usr/bin/env python

import argparse
from env import make_env
import random

DIR_NAME = './data/rollout/'

WAYPOINT_REWARD_MULTIPLIER = 1.0
TIME_PENALTY = -0.1

def main(args):

    env_name = args.env_name
    total_episodes = args.total_episodes
    time_steps = args.time_steps
    render = args.render
    action_refresh_rate = args.action_refresh_rate


    for i in range(total_episodes):
        episode_id = random.randint(0, 2**31 - 1)
        filename = DIR_NAME + str(episode_id) + ".npz"
        track_name = 'track'+str(random.randint(1,20))

        env = make_env('SingleRacecar', track_name=track_name, waypoint_reward=WAYPOINT_REWARD_MULTIPLIER, time_reward=TIME_PENALTY)

        observation = env.reset()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=('Create new training data'))
    parser.add_argument('env_name', type=str, help='name of environment')
    parser.add_argument('--total_episodes', type=int, default=200,
                        help='total number of episodes to generate per worker')
    parser.add_argument('--time_steps', type=int, default=300,
                        help='how many timesteps at start of episode?')
    parser.add_argument('--render', default=0, type=int,
                        help='render the env as data is generated')
    parser.add_argument('--action_refresh_rate', default=20, type=int,
                        help='how often to change the random action, in frames')
    
    args = parser.parse_args()
    main(args)