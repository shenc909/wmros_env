#!/usr/bin/env python

from gazebo_gym.gazebo_racecar_env import GazeboRaceCarEnv
from mpi4py import MPI

class SingeRacecarWrapper:

    def __init__(self):
        if (rank == 0):
            leader()
        else:
            follower()
    
    def leader(self):
        pass

    def follower(self):
        env = GazeboRaceCarEnv()