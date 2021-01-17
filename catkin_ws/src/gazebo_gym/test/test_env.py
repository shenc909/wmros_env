#!/usr/bin/env python

from gazebo_gym.gazebo_racecar_env import GazeboRaceCarEnv
import time
import rospy
import random
import math
import cv2
import numpy as np

step_size = 0.01
track_name = 'track10'
speed_multiplier = 0.7
angle_multiplier = 0.5
steps = 300

def main():
    env = GazeboRaceCarEnv(step_size, track_name)
    obs = env.reset()
    cv2.imshow('preview', obs)
    cv2.waitKey(0)
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
            cv2.namedWindow('observation', cv2.WINDOW_KEEPRATIO)
            cv2.imshow('observation', state)
            cv2.resizeWindow('observation', 300, 300)
            cv2.waitKey(1)
        else:
            rospy.logerr('NO STATE RECEIVED')
        print(f'Reward: {reward}')
        # input("Hit enter to step env")
    # time.sleep(20)
    # input("Hit enter to close env")
    cv2.destroyAllWindows()
    rospy.loginfo('Environment Done')
    env._close()
    # rospy.signal_shutdown('End of env')

if __name__ == "__main__":
    try:
        for i in range(2):
            main()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupted")