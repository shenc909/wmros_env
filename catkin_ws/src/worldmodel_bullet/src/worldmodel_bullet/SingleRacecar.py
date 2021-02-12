#!/usr/bin/env python

from worldmodel_bullet.BulletEnv import BulletEnv
import rospy
from worldmodel_bullet.SimulationManager import SimulationManager, SimulatedCar
from geometry_msgs.msg import Point, Quaternion, Pose
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import gym
from gym.utils import seeding
from gym import spaces
import numpy as np

STATE_W = 64
STATE_H = 64

MIN_SPEED = 0
MAX_SPEED = 3

class SingleRacecar(BulletEnv):

    def __init__(self, waypoint_threshold=1.0, waypoint_reward_mult=1.0, time_reward=-1.0, render_mode='headless'):
        
        BulletEnv.__init__(self)

        self.render_mode = render_mode

        self.sm = SimulationManager(render_mode=self.render_mode)
        rospy.init_node('bullet_gym', anonymous=True)

        self.position_pub = rospy.Publisher('/ackermann_vehicle/position', Pose, queue_size=1)
        self.reward_pub = rospy.Publisher('/ackermann_vehicle/reward', Float32, queue_size=1)
        self.camera_pub = rospy.Publisher('/ackermann_vehicle/camera0/image_raw', Image, queue_size=1)

        self.bridge = CvBridge()

        self.action_space = spaces.Box(low=np.array([0, -1]), high=np.array([1,1]), shape=(2,)) #speed, steering angle
        self.reward_range = (-np.inf, np.inf)
        self.observation_space = spaces.Box(low=0.0, high=1.0, shape=(STATE_H, STATE_W, 3), dtype=np.float32)
        self.reward = 0

        # self.sc = SimulatedCar(start_position=[0,0,0.5])


    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def seed(self, seed=None):
        self._seed(seed)
    
    def reset(self):
        self.sm.reset_simulation()
        random_track_num = np.random.randint(1, 20)
        random_track_name = f'track{random_track_num}'
        self.sm.spawn_track(random_track_name)
        self.sc = SimulatedCar(start_position=[0,0,0], render_mode=self.render_mode)
        img = self.sc.get_image()
        
        img_msg = self.bridge.cv2_to_imgmsg(img)
        self.camera_pub.publish(img_msg)

        return img