#!/usr/bin/env python

import rospy
import random
import math
from ackermann_msgs.msg import AckermannDrive

pub_rate = 30
change_rate = 0.2
speed_multiplier = 0.7
angle_multiplier = 0.5

pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
rospy.init_node('ackermann_commander')
r = rospy.Rate(pub_rate) # 1hz

i = 0

while not rospy.is_shutdown():
    change = pub_rate/change_rate
    msg = AckermannDrive()
    if i%change == 0:
        steering_angle = (random.random() - 0.5) * angle_multiplier * math.pi
        speed = random.random() * speed_multiplier
    msg.steering_angle = steering_angle
    msg.speed = speed
    pub.publish(msg)
    i = i + 1
    r.sleep()