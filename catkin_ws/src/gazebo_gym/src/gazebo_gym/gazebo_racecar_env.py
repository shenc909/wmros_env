#!/usr/bin/env python

import rospy
import gym
from gym import spaces
from gazebo_gym import gazebo_env
from std_srvs.srv import Empty
from worldmodel_interface.srv import GetSingleReward, GetSingleRewardResponse
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Image
from gym.utils import seeding
import numpy as np
from cv_bridge import CvBridge
import cv2
import time
import importlib

importlib.reload(rospy)


STATE_W = 64
STATE_H = 64

MIN_SPEED = 0
MAX_SPEED = 1.5
MIN_ANGLE = -np.pi/2 * 0.75
MAX_ANGLE = np.pi/2 * 0.75

class GazeboRaceCarEnv(gazebo_env.GazeboEnv):

    """
    Create a single car Gazebo Environment

    Keyword Arguments:

    step_size -- time (in seconds) between steps

    track_name -- name of track to load

    waypoint_threshold -- distance (in meters) before waypoint is considered reached

    waypoint_reward_mult -- reward gained per waypoint multiplier (1000/N, N is number of waypoints)

    time_reward -- reward gained per second
    """

    def __init__(self, step_size=0.1, track_name='track1', waypoint_threshold=1.0, waypoint_reward_mult=1.0, time_reward=-1.0, gazebo_gui=False):
        # Launch the simulation with the given launchfile name
        self.step_size = step_size
        gazebo_env.GazeboEnv.__init__(self, "GazeboRacecarEnv.launch", \
            launch_args=[\
                f'world_name:={track_name}',
                f'waypoint_threshold:={waypoint_threshold}',
                f'waypoint_reward_mult:={waypoint_reward_mult}',
                f'time_reward:={time_reward}',
                f'gazebo_gui:={str(gazebo_gui).lower()}'])
        rospy.init_node('gym', anonymous=True)
        self.ackermann_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDrive, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reward_proxy = rospy.ServiceProxy('/ackermann_vehicle/reward', GetSingleReward)
        self.reward_reset_proxy = rospy.ServiceProxy('/ackermann_vehicle/reward_reset', Empty)

        # cv_bridge/gazebo camera plugin produces a single channel/weirdo image if there isn't at least one subscriber
        rospy.Subscriber('/ackermann_vehicle/camera1/image_raw', Image, self._camera_subscriber)

        self.action_space = spaces.Box(low=np.array([MIN_SPEED, MIN_ANGLE]), high=np.array([MAX_SPEED,MAX_ANGLE]), shape=(2,)) #speed, steering angle
        self.reward_range = (-np.inf, np.inf)
        self.observation_space = spaces.Box(low=0, high=255, shape=(STATE_H, STATE_W, 3), dtype=np.uint8)
        self.reward = 0
        self.bridge = CvBridge()
        self.track_name = track_name
        
        self.image_msg = None

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
        while data is None and not rospy.is_shutdown():
            try:
                # data = rospy.wait_for_message('/ackermann_vehicle/camera1/image_raw', Image, timeout=5)
                while self.image_msg is None:
                    rospy.logwarn_throttle(5, 'No camera updates')
                data = self.image_msg
                obs = self._imageMsgToCv2(data)
                self.image_msg = None
            except:
                rospy.logerr_throttle(1, 'No data found')

        self._pauseGazebo()
        
        state = obs
        reward, done = self._rewardCalc()

        return state, reward, done, {}
    
    def reset(self):
        self._resetGazebo()
        rospy.loginfo('Sleeping for 3s while gazebo loads')
        time.sleep(3)

        self._resumeGazebo()

        #read camera data
        data = None
        obs = None
        while data is None and not rospy.is_shutdown():
            try:
                # data = rospy.wait_for_message('/ackermann_vehicle/camera1/image_raw', Image, timeout=10)
                while self.image_msg is None:
                    rospy.logwarn_throttle(5, 'No camera updates')
                data = self.image_msg
                obs = self._imageMsgToCv2(data)
                self.image_msg = None
            except rospy.ROSException:
                rospy.logfatal('Cannot establish camera image (check tf tree?)')

        self._pauseGazebo()

        state = obs

        # state, _, _, _ = self.step([0,0])
        self.reward = 0
        self._resetReward()

        return state
    
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    def _rewardCalc(self):
        rospy.wait_for_service('/ackermann_vehicle/reward')
        try:
            response = self.reward_proxy()
            return response.reward, response.done

        except (rospy.ServiceException) as e:
            print ("/ackermann_vehicle/reward service call failed")
            return None, None

    def _imageMsgToCv2(self, msg):
        img = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough'), cv2.COLOR_BGR2RGB)
        # resize image to VAE input size
        img = cv2.resize(img, (STATE_W, STATE_H))
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
    
    def _resetReward(self):

        rospy.wait_for_service('/ackermann_vehicle/reward_reset')
        try:
            self.reward_reset_proxy()
        except (rospy.ServiceException) as e:
            print("/ackermann_vehicle/reward_reset call failed")
    
    def _camera_subscriber(self,msg):
        self.image_msg = msg

    def __del__(self):
        print('Being destroyed rn')