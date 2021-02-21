#!/usr/bin/env python

from BulletEnv import BulletEnv
from SimulationManager import SimulationManager, SimulatedCar
from RewardCalculator import RewardCalculator
import cv2
import gym
from gym.utils import seeding
from gym import spaces
import numpy as np

STATE_W = 64
STATE_H = 64

MIN_SPEED = 0
MAX_SPEED = 50

MAX_FORCE = 30

DEFAULT_SIM_FREQ = 240

STABILISE_TIMESTEP = 50

DEFAULT_SPAWN_HEIGHT = 0.1

ROS_ENABLE = False

if ROS_ENABLE:
    import rospy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Point, Quaternion, Pose
    from std_msgs.msg import Float32
    from sensor_msgs.msg import Image

class SingleRacecar(BulletEnv):

    def __init__(self, waypoint_threshold=1.0, waypoint_reward_multi=1.0, timestep_reward=-1.0, render_mode='headless', step_freq=240):
        
        BulletEnv.__init__(self)

        self.render_mode = render_mode
        self.waypoint_threshold = waypoint_threshold
        self.waypoint_reward_multi = waypoint_reward_multi
        self.timestep_reward = timestep_reward
        self.step_freq = step_freq

        self.step_num = self._steps_calc(self.step_freq)

        self.sm = SimulationManager(render_mode=self.render_mode)
        
        if ROS_ENABLE:
            rospy.init_node('bullet_gym', anonymous=True)
            self.pose_pub = rospy.Publisher('/ackermann_vehicle/pose', Pose, queue_size=10)
            self.reward_pub = rospy.Publisher('/ackermann_vehicle/reward', Float32, queue_size=10)
            self.camera_pub = rospy.Publisher('/ackermann_vehicle/camera0/image_raw', Image, queue_size=10)

            self.bridge = CvBridge()

        self.action_space = spaces.Box(low=np.array([0, -1]), high=np.array([1,1]), shape=(2,)) #speed, steering angle
        self.reward_range = (-np.inf, np.inf)
        self.observation_space = spaces.Box(low=0.0, high=1.0, shape=(STATE_H, STATE_W, 3), dtype=np.float32)
        self.reward = 0

        # self.sc = SimulatedCar(start_position=[0,0,0.5])

    def seed(self, seed=None):
        self._seed(seed)
    
    def reset(self):
        if ROS_ENABLE:
            self._rospy_check()

        self.sm.reset_simulation()
        random_track_num = np.random.randint(1, 20)
        random_track_name = f'track{random_track_num}'
        self.sm.spawn_track(random_track_name)

        self.rc = RewardCalculator(track_name=random_track_name, waypoint_reward_multi=1, timestep_reward=-0.1, threshold_distance=self.waypoint_threshold)

        self.sc = SimulatedCar(start_position=[0,0,DEFAULT_SPAWN_HEIGHT], render_mode=self.render_mode)

        # timesteps to stabilise
        for i in range(STABILISE_TIMESTEP):
            # print('stabbing')
            self.sm.step_simulation()

            img = self.sc.get_image(image_width=STATE_W, image_height=STATE_H)

            self.state = img
        
        # print('stab done')
        
        if ROS_ENABLE:
            img_msg = img * 255
            img_msg = img_msg.astype(np.uint8)
            img_msg = self.bridge.cv2_to_imgmsg(img_msg, encoding='rgb8')
            self.camera_pub.publish(img_msg)

        return img
    
    def step(self, action):
        if ROS_ENABLE:
            self._rospy_check()

        speed = action[0]
        steering = action[1]

        speed = speed * (MAX_SPEED - MIN_SPEED) + MIN_SPEED

        self.sc.set_speed(wheel_vel=speed, max_force=MAX_FORCE)
        self.sc.set_steering(steering_angle=steering)

        for i in range(self.step_num):
            self.sm.step_simulation()

        state = self.sc.get_image(image_width=STATE_W, image_height=STATE_H)
        self.state = state

        if ROS_ENABLE:
            img_msg = state * 255
            img_msg = img_msg.astype(np.uint8)
            img_msg = self.bridge.cv2_to_imgmsg(img_msg, encoding='rgb8')
            self.camera_pub.publish(img_msg)

        pos, ori = self._getPose()

        reward, done = self.rc.get_reward(pos)

        return state, reward, done, {}
    
    def close(self):
        if self.render_mode in ['headless', 'human']:
            cv2.destroyAllWindows()

        self.sm.close()
        if ROS_ENABLE:
            self._close()
    
    def render(self):

        if self.render_mode in ['headless', 'human']:
            cv2.namedWindow('observation', cv2.WINDOW_KEEPRATIO)
            obs = cv2.cvtColor(self.state, cv2.COLOR_RGB2BGR)
            cv2.imshow('observation', obs)
            cv2.resizeWindow('observation', 300, 300)
            cv2.waitKey(1)
            # cv2.imshow('state', self.state)
            # cv2.waitKey(1)
        else:
            print(f'Invalid render mode: {self.render_mode}, needs to be ["headless", "human"]')
    
    def _getPose(self):
        pos = self.sc.get_position()
        ori = self.sc.get_orientation(quaternion=True)

        if ROS_ENABLE:
            pose_msg = Pose()

            pos_msg = Point()
            pos_msg.x = pos[0]
            pos_msg.y = pos[1]
            pos_msg.z = pos[2]

            ori_msg = Quaternion()
            ori_msg.x = ori[0]
            ori_msg.y = ori[1]
            ori_msg.z = ori[2]
            ori_msg.w = ori[3]

            pose_msg.position = pos_msg
            pose_msg.orientation = ori_msg

            self.pose_pub.publish(pose_msg)

        return pos, ori
    
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    def _steps_calc(self, time_step):

        steps = DEFAULT_SIM_FREQ/self.step_freq

        step_num = int(steps)

        if not steps.is_integer():
            print(f'Warning: Chosen step freq is not compatible with {DEFAULT_SIM_FREQ}Hz default calc freq')
            print(f'Real step freq will be {DEFAULT_SIM_FREQ / self.step_num}')
        
        return step_num
    
    if ROS_ENABLE:
        def _rospy_check(self):

            if rospy.is_shutdown():
                raise KeyboardInterrupt