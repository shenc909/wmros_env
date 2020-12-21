#!/usr/bin/env python

import tf
import rospy
import rospkg
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty
from worldmodel_interface.srv import GetSingleReward, GetSingleRewardResponse
import numpy as np

COORD_PATH = 'gym_assets/track_coords'
PKG_NAME = 'gazebo_gym'

class RewardPublisher:

    def __init__(self):
        self.waypoint_index = 0
        self.reward = 0
        self.done = False

        rospy.init_node('car_tf_listener')
        self.listener = tf.TransformListener()

        # Get parameters
        self.track_name = rospy.get_param('~track_name', "track1")
        self.car_namespace = rospy.get_param('~namespace', '/ackermann_vehicle')
        self.waypoint_reward_mult = rospy.get_param('~waypoint_reward_mult', 10)
        self.time_reward = rospy.get_param('~time_reward', -1)
        self.threshold_distance = rospy.get_param('~threshold_distance', 1)
        self.rospack = rospkg.RosPack()

        # Get time for tme reward calc
        self.last_time = rospy.get_time()

        rospy.loginfo(f'{self.track_name}, NS: {self.car_namespace}, wpr:{self.waypoint_reward_mult}, tr:{self.time_reward}, thresh:{self.threshold_distance}')

        # Load waypoints for track
        self._loadWaypoints()
        # Broadcast service to get reward
        self.reward_service = rospy.Service(f'reward', GetSingleReward, self.getSingleReward)
        self.reset_service = rospy.Service(f'reward_reset', Empty, self.reset)
        

    def rewardSpin(self):

        if self.waypoint_index >= len(self.waypoints):
            self.done = True
            return
        
        self.curr_time = rospy.get_time()
        elapsed = self.curr_time - self.last_time
        self.reward += self.time_reward * elapsed

        try:
            position, quarternion = self.listener.lookupTransform(self.car_namespace+'/base_link', 'world', rospy.Time())
            car_coordinate = np.array([position[0], position[1]])

            target_waypoint = self.waypoints[self.waypoint_index]
            car2point_vector = target_waypoint - car_coordinate

            dist = np.linalg.norm(car2point_vector)

            if dist < self.threshold_distance:
                self.reward += self.waypoint_reward_mult * 1000/(len(self.waypoints))
                self.waypoint_index += 1

        except tf.LookupException:
            rospy.logerr_throttle(10, f'Lookup from {self.car_namespace} to world not found')
        
        # rospy.loginfo_throttle(0.5, rospy.get_time())
        
        self.last_time = self.curr_time


    def getSingleReward(self, req):
        response_dict = {'reward': float(self.reward), 'done': self.done}
        # Reset internal reward to zero
        self.reward = 0
        rospy.loginfo('Step service called')
        return response_dict
    
    def reset(self, req):
        self.waypoint_index = 0
        self.reward = 0
        self.last_time = rospy.get_time()
        return []
    
    def _loadWaypoints(self):
        path = self.rospack.get_path(PKG_NAME)
        self.waypoints = np.load(f'{path}/src/{PKG_NAME}/{COORD_PATH}/{self.track_name}.dxf.npy')

if __name__ == "__main__":

    rp = RewardPublisher()

    try:
        while not rospy.is_shutdown():
            rp.rewardSpin()

    except rospy.ROSInterruptException:
        pass