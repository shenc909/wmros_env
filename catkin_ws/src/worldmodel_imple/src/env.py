#!/usr/bin/env python

from gazebo_gym.gazebo_racecar_env import GazeboRaceCarEnv

def make_env(env_name, **kwargs):
    if env_name == 'SingleRacecar':
        """
        Create a single car Gazebo Environment

        Keyword Arguments:

        step_size -- time (in seconds) between steps

        track_name -- name of track to load

        waypoint_threshold -- distance (in meters) before waypoint is considered reached

        waypoint_reward -- reward gained per waypoint

        time_reward -- reward gained per second
        """
        env = GazeboRaceCarEnv(**kwargs)
        return env
    else:
        raise NameError(f'{env_name} environment does not exist')

if __name__ == "__main__":
    make_env('SingleRacecar')