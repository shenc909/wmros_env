#!/usr/bin/env python

from gazebo_racecar_env import GazeboRaceCarEnv
import time
import rospy
import random
import math

step_size = 1
track_number = 5
speed_multiplier = 0.7
angle_multiplier = 0.5
steps = 100

def main():
    env = GazeboRaceCarEnv(step_size, track_number)
    env.reset()
    env._render()
    for i in range(steps):
        # steering_angle = (random.random() - 0.5) * angle_multiplier * math.pi
        # speed = random.random() * speed_multiplier
        action = env.action_space.sample()
        env.step(action)
        input("Hit enter to step env")
    # time.sleep(20)
    input("Hit enter to close env")
    env._close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupted")