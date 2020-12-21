#!/usr/bin/env python

from gazebo_gym.gazebo_racecar_env import GazeboRaceCarEnv
import time
import rospy
import random
import math
import cv2
import numpy as np

step_size = 0.1
track_name = 'track5'
speed_multiplier = 0.7
angle_multiplier = 0.5
steps = 300

def main():
    env = GazeboRaceCarEnv(step_size, track_name)
    env.reset()
    env._render()
    done = False
    for i in range(steps):
        # steering_angle = (random.random() - 0.5) * angle_multiplier * math.pi
        # speed = random.random() * speed_multiplier
        action = env.action_space.sample()
        state, reward, done, debug_dict = env.step(action)
        if done:
            break
        # rospy.loginfo(state.tolist())
        if state is not None:
            rospy.loginfo(np.shape(state))
            # cv2.imshow('item', state)
            # cv2.waitKey(0)
        else:
            rospy.logerr('NO STATE RECEIVED')
        print(f'Reward: {reward}')
        # input("Hit enter to step env")
    # time.sleep(20)
    # input("Hit enter to close env")
    rospy.loginfo('Environment Done')
    env._close()
    rospy.signal_shutdown('End of env')

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupted")