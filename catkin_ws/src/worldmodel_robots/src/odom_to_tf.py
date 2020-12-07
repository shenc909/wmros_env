#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
from tf2_msgs.msg import TFMessage

def handle_car_odom(msg):
    br = TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     tfPrefix + "/" + msg.child_frame_id,
                     "world")

if __name__ == '__main__':
    rospy.init_node('odom_to_tf')
    tfPrefix = rospy.get_param("~tf_prefix")
    rospy.Subscriber('/ground_truth/state',
                     Odometry,
                     handle_car_odom)
    rospy.spin()