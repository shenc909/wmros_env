#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

def main():
    rospy.init_node('camera_subscriber', anonymous=True)
    camera_image_topic = rospy.get_param('~camera_image_topic', "camera1/image_raw")
    car_namespace = rospy.get_param('~namespace', '/ackermann_vehicle')
    rospy.Subscriber(f'/{car_namespace}/{camera_image_topic}', Image, camera_callback)

    rospy.spin()

def camera_callback(msg):
    pass

if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass