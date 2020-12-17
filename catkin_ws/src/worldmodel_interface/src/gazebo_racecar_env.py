#!/usr/bin/env python

import rospy
import gym
from gym import spaces
import gazebo_env
from std_srvs.srv import Empty
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Image
from gym.utils import seeding
import numpy as np

class GazeboRaceCarEnv(gazebo_env.GazeboEnv):
    """Gazebo racecar environment"""

    def __init__(self, step_size, map_number):
        # Launch the simulation with the given launchfile name
        self.step_size = step_size
        gazebo_env.GazeboEnv.__init__(self, "GazeboRacecarEnv.launch", launch_args=[f'world_name:=track{map_number}'])
        self.vel_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDrive, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.action_space = spaces.Box(low=10, high=20, shape=(2,1)) #speed, steering angle
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def step(self, action):
        # Execute one time step within the environment
        pass
    
    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        #read laser data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/ackermann_vehicle/camera1/image_raw', Image, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state = None

        return state
    
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]