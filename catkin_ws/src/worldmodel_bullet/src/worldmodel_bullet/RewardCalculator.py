#!/usr/bin/env python

# import rospy
# import rospkg
from std_msgs.msg import Float64, Bool
import numpy as np
import os
from pathlib import Path

COORD_PATH = 'assets/track_coords'
PKG_NAME = 'worldmodel_bullet'

class RewardCalculator:

    def __init__(self, track_name='track1', waypoint_reward_multi=10,timestep_reward=-1,threshold_distance=1):
        self.reward = 0
        self.done = False

        # Get parameters
        self.track_name = track_name
        self.waypoint_reward_multi = waypoint_reward_multi
        self.timestep_reward = timestep_reward
        self.threshold_distance = threshold_distance
        # self.rospack = rospkg.RosPack()

        # Load waypoints for track
        self._loadWaypoints()
        self.waypoint_check = np.full(len(self.waypoints), False)
    
    def get_reward(self, car_position):

        self.reward += self.timestep_reward

        # car_position = car_position[0:2]

        for idx, target_waypoint in zip(range(len(self.waypoints)), self.waypoints):
            # target_waypoint = self.waypoints[self.waypoint_index]
            
            target_waypoint = np.hstack((target_waypoint, np.array([0])))
            # print(np.shape(target_waypoint))
            car2point_vector = target_waypoint - car_position

            dist = np.linalg.norm(car2point_vector)

            if dist < self.threshold_distance:
                if self.waypoint_check[idx] == False:
                    self.reward += self.waypoint_reward_multi * 1000/(len(self.waypoints))
                    # self.waypoint_index += 1
                    self.waypoint_check[idx] = True
                    break
            
            if all(self.waypoint_check):
                self.done = True
                # return self.reward, self.done
        
        return_reward = self.reward
        self.reward = 0
        return return_reward, self.done
    
    def reset(self):
        # self.waypoint_index = 0
        self.waypoint_check = np.full(len(self.waypoints), False)
        self.reward = 0
        return []
    
    def _loadWaypoints(self):
        # path = self.rospack.get_path(PKG_NAME)
        path = os.path.join(Path(__file__).resolve().parent, '../../')
        self.waypoints = np.load(f'{path}/src/{PKG_NAME}/{COORD_PATH}/{self.track_name}.dxf.npy')