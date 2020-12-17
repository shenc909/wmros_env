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
from cv_bridge import CvBridge
import cv2

STATE_W = 64
STATE_H = 64

class GazeboRaceCarEnv(gazebo_env.GazeboEnv):
    """Gazebo racecar environment"""

    def __init__(self, step_size, track_number):
        # Launch the simulation with the given launchfile name
        self.step_size = step_size
        gazebo_env.GazeboEnv.__init__(self, "GazeboRacecarEnv.launch", launch_args=[f'world_name:=track{track_number}'])
        self.ackermann_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDrive, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.action_space = spaces.Box(low=np.array([-5, -np.pi/2]), high=np.array([5,np.pi/2]), shape=(2,)) #speed, steering angle
        self.reward_range = (-np.inf, np.inf)
        self.observation_space = spaces.Box(low=0, high=255, shape=(STATE_H, STATE_W, 3), dtype=np.uint8)
        self.reward = 0
        self.bridge = CvBridge()
        self.track_number = track_number

        self._seed()

    def step(self, action):
        self._resumeGazebo()

        ackermann_cmd = AckermannDrive()
        ackermann_cmd.speed = action[0]
        ackermann_cmd.steering_angle = action[1]
        self.ackermann_pub.publish(ackermann_cmd)

        rospy.sleep(self.step_size)

        data = None
        obs = None
        done = False
        while data is None:
            try:
                data = rospy.wait_for_message('/ackermann_vehicle/camera1/image_raw', Image, timeout=5)
                obs = self._imageMsgToCv2(data)
            except:
                pass

        self._pauseGazebo()
        
        if obs != None:
            done = True
        
        state = obs
        reward = self._rewardCalc()

        return state, reward, done, {}
    
    def reset(self):
        self._resetGazebo()

        self._resumeGazebo()

        #read camera data
        data = None
        obs = None
        while data is None:
            try:
                data = rospy.wait_for_message('/ackermann_vehicle/camera1/image_raw', Image, timeout=5)
                obs = self._imageMsgToCv2(data)
            except:
                pass

        self._pauseGazebo()

        state = obs
        self.reward = 0

        return state
    
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    def _rewardCalc(self):
        pass

    def _imageMsgToCv2(self, msg):
        img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(data,desired_encoding='rgb8'), cv2.COLOR_BGR2RGB)
        return img
    
    def _resetGazebo(self):
        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")
    
    def _pauseGazebo(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
    
    def _resumeGazebo(self):
        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")