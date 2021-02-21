#!/usr/bin/env python

import rospy
import roslaunch
import rospkg
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
from controller_manager_msgs.srv import ListControllers

# importlib.reload(rospy)


STATE_W = 64
STATE_H = 64

MIN_SPEED = 0
MAX_SPEED = 3
MIN_ANGLE = -np.pi/2 * 0.75
MAX_ANGLE = np.pi/2 * 0.75

class GazeboRaceCarEnv(gazebo_env.GazeboEnv):

    """
    Create a single car Gazebo Environment

    Keyword Arguments:

    step_size -- time (in seconds) between steps

    waypoint_threshold -- distance (in meters) before waypoint is considered reached

    waypoint_reward_mult -- reward gained per waypoint multiplier (1000/N, N is number of waypoints)

    time_reward -- reward gained per second
    """

    def __init__(self, step_size=0.1, waypoint_threshold=1.0, waypoint_reward_mult=1.0, time_reward=-1.0, gazebo_gui=False):

        if rospy.is_shutdown():
            raise(KeyboardInterrupt)
        
        # Launch the simulation with the given launchfile name
        self.step_size = step_size
        # self.rospack = rospkg.RosPack()
        gazebo_env.GazeboEnv.__init__(self, "GazeboRacecarEnv.launch", \
            # launch_args=[\
            #     f'world_name:={track_name}',
            #     f'waypoint_threshold:={waypoint_threshold}',
            #     f'waypoint_reward_mult:={waypoint_reward_mult}',
            #     f'time_reward:={time_reward}',
            #     f'gazebo_gui:={str(gazebo_gui).lower()}']
            )

        # track_name = self._random_track_world()
        # self._general_args = [f'world_name:={track_name}']
        self._world_loader_args = [f'gazebo_gui:={str(gazebo_gui).lower()}']
        self._support_publishers_args = [\
            f'waypoint_threshold:={waypoint_threshold}',
            f'waypoint_reward_mult:={waypoint_reward_mult}',
            f'time_reward:={time_reward}']
        
        # print('Starting env launchers')
        # self._load_world(self._general_args, self._world_loader_args)
        # self._spawn_car(self._general_args, [])
        # self._start_support_publishers(self._general_args, self._support_publishers_args)

        rospy.init_node('gym', anonymous=True)
        self.ackermann_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDrive, queue_size=1)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty, persistent=True)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty, persistent=True)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reward_proxy = rospy.ServiceProxy('/ackermann_vehicle/reward', GetSingleReward, persistent=True)
        self.reward_reset_proxy = rospy.ServiceProxy('/ackermann_vehicle/reward_reset', Empty)

        self.list_ctrlrs = rospy.ServiceProxy("/ackermann_vehicle/controller_manager/list_controllers", ListControllers)

        # cv_bridge/gazebo camera plugin produces a single channel/weirdo image if there isn't at least one subscriber
        rospy.Subscriber('/ackermann_vehicle/camera1/image_raw', Image, self._camera_subscriber, queue_size = 1)

        self.action_space = spaces.Box(low=np.array([0, -1]), high=np.array([1,1]), shape=(2,)) #speed, steering angle
        self.reward_range = (-np.inf, np.inf)
        self.observation_space = spaces.Box(low=0.0, high=1.0, shape=(STATE_H, STATE_W, 3), dtype=np.float32)
        self.reward = 0
        self.bridge = CvBridge()
        # self.track_name = track_name
        
        self.image_msg = None

        self._seed()

    def step(self, action):
        
        if rospy.is_shutdown():
            raise(KeyboardInterrupt)

        self._resumeGazebo()

        ackermann_cmd = AckermannDrive()
        ackermann_cmd.speed = action[0] * (MAX_SPEED - MIN_SPEED)
        ackermann_cmd.steering_angle = action[1] * MAX_ANGLE
        self.ackermann_pub.publish(ackermann_cmd)

        rospy.sleep(self.step_size)

        data = None
        obs = None
        done = False
        while data is None and not rospy.is_shutdown():
            try:
                # data = rospy.wait_for_message('/ackermann_vehicle/camera1/image_raw', Image, timeout=5)
                while self.image_msg is None:
                    rospy.logwarn_throttle(5, 'No camera updates in step')
                data = self.image_msg
                obs = self._imageMsgToCv2(data)
                obs = cv2.cvtColor(obs, cv2.COLOR_BGR2RGB)
                obs = np.float32(obs) / 255
                self.image_msg = None
            except:
                rospy.logerr_throttle(1, 'No data found')

        self._pauseGazebo()
        
        state = obs
        reward, done = self._rewardCalc()

        return state, reward, done, {}
    
    def reset(self):
        
        if rospy.is_shutdown():
            raise(KeyboardInterrupt)

        try:
            print('Stopping World')
            self._world_loader_parent.shutdown()
            print('Stopping Car')
            self._car_spawner_parent.shutdown()
            print('Stopping Support Publishers')
            self._support_publishers_parent.shutdown()
        except AttributeError:
            rospy.logwarn('Cannot kill nonexistent launchers, skipping...')

        track_name = self._random_track_world()
        self._general_args = [f'world_name:={track_name}']

        print('Starting env launchers')
        rospy.set_param('/use_sim_time', True)
        self._load_world(self._general_args, self._world_loader_args)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty, persistent=True)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty, persistent=True)
        self.reward_proxy = rospy.ServiceProxy('/ackermann_vehicle/reward', GetSingleReward, persistent=True)

        self._resetGazebo()
        # rospy.loginfo('Sleeping for 3s while gazebo loads')
        time.sleep(3)

        self._spawn_car(self._general_args, [])
        self._start_support_publishers(self._general_args, self._support_publishers_args)

        self.list_ctrlrs.wait_for_service()
        # rospy.loginfo('Sleeping for 5s for controllers to spawn and be up')
        # time.sleep(5)
        self._wait_for_ctrlr()

        self._resumeGazebo()

        #read camera data
        data = None
        obs = None
        while data is None and not rospy.is_shutdown():
            try:
                # data = rospy.wait_for_message('/ackermann_vehicle/camera1/image_raw', Image, timeout=10)
                while self.image_msg is None:
                    rospy.logwarn_throttle(5, 'No camera updates in reset')
                data = self.image_msg
                obs = self._imageMsgToCv2(data)
                obs = cv2.cvtColor(obs, cv2.COLOR_BGR2RGB)
                obs = np.float32(obs) / 255
                self.image_msg = None
            except rospy.ROSException:
                rospy.logfatal('Cannot establish camera image (check tf tree?)')

        self._pauseGazebo()

        state = obs

        # state, _, _, _ = self.step([0,0])
        self.reward = 0
        self._resetReward()

        return state
    
    def close(self):
        print('Stopping World')
        self._world_loader_parent.shutdown()
        print('Stopping Car')
        self._car_spawner_parent.shutdown()
        print('Stopping Support Publishers')
        self._support_publishers_parent.shutdown()
        self._close()
    
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
    
    def _load_world(self, general_args, world_loader_args):
        
        self._world_loader_path = self.get_fullpath("WorldLoader.launch")
        self._world_loader_uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        cli_args = [self._world_loader_path] + general_args + world_loader_args
        roslaunch_args = general_args + world_loader_args
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        self._world_loader_parent = roslaunch.parent.ROSLaunchParent(self._world_loader_uuid, roslaunch_files=roslaunch_file, show_summary=False)
        self._world_loader_parent.start()
        print('World Loader Started')

    def _spawn_car(self, general_args, car_spawner_args):
        self._car_spawner_path = self.get_fullpath("CarSpawner.launch")
        self._car_spawner_uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        cli_args = [self._car_spawner_path] + general_args + car_spawner_args
        roslaunch_args = general_args + car_spawner_args
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        self._car_spawner_parent = roslaunch.parent.ROSLaunchParent(self._car_spawner_uuid, roslaunch_files=roslaunch_file, show_summary=False)
        self._car_spawner_parent.start()
        print('Car Spawner Started')
    
    def _start_support_publishers(self, general_args, support_publishers_args):
        self._support_publishers_path = self.get_fullpath("SupportPublishers.launch")
        self._support_publishers_uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        cli_args = [self._support_publishers_path] + general_args + support_publishers_args
        roslaunch_args = general_args + support_publishers_args
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        self._support_publishers_parent = roslaunch.parent.ROSLaunchParent(self._support_publishers_uuid, roslaunch_files=roslaunch_file, show_summary=False)
        self._support_publishers_parent.start()
        print('Support Publishers Started')
    
    def _random_track_world(self):
        rand_int = self.np_random.randint(1, 21)
        track_name = 'track' + str(rand_int)
        return track_name
    
    def seed(self, seed=None):
        self._seed(seed)

    def _wait_for_ctrlr(self):
    # Wait for the specified controller to be in the "running" state.
    # Commands can be lost if they are published before their controller is
    # running, even if a latched publisher is used.

        while True:
            response = self.list_ctrlrs()
            for ctrlr in response.controller:
                if ctrlr.state == "running":
                    return
                rospy.sleep(0.1)