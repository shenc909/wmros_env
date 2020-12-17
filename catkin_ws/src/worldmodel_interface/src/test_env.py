#!/usr/bin/env python

from gazebo_racecar_env import GazeboRaceCarEnv
import time
import rospy

def main():
    env = GazeboRaceCarEnv(0.1, 5)
    env.reset()
    env._render()
    # time.sleep(20)
    input("Hit enter to close env")
    env._close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.signal_shutdown()