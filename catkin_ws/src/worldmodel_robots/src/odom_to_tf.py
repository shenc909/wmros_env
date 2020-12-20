#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf import TransformBroadcaster

from gazebo_msgs.msg import ModelStates

def handle_car_odom(msg):
    br = TransformBroadcaster()
    # t = TransformStamped()
    
    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "world"
    # t.child_frame_id = tfPrefix + "/" + msg.child_frame_id

    # t.transform.translation.x = msg.pose.pose.position.x
    # t.transform.translation.y = msg.pose.pose.position.y
    # t.transform.translation.z = msg.pose.pose.position.z

    # t.transform.rotation.x = msg.pose.pose.orientation.x
    # t.transform.rotation.y = msg.pose.pose.orientation.y
    # t.transform.rotation.z = msg.pose.pose.orientation.z
    # t.transform.rotation.w = msg.pose.pose.orientation.w

    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     tfPrefix + "/" + msg.child_frame_id,
                     "world")

    # br.sendTransform(t)

def handle_model_state(msg):
    br = TransformBroadcaster()

    model_names = msg.name
    # rospy.logerr_throttle(1,f'{str(model_names)}')

    try:
        # idx = model_names.index(tfPrefix)

        idx = -1

        for i in range(len(model_names)):
            # rospy.logerr_throttle_identical(1,f'{tfPrefix}:{model_names[i]}')
            if tfPrefix == model_names[i]:
                idx = i
                # rospy.logwarn('FOUND')
                break

        pose = msg.pose[idx]
    
        br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                     (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                     rospy.Time.now(),
                     tfPrefix + "/" + msg.child_frame_id,
                     "world")
    except:
        rospy.logerr_throttle(1, f'Cannot find Gazebo model state {tfPrefix}')

if __name__ == '__main__':
    rospy.init_node('odom_to_tf')
    tfPrefix = rospy.get_param("~tf_prefix")
    rospy.Subscriber('/ground_truth/state',
                     Odometry,
                     handle_car_odom)
    rospy.Subscriber('/gazebo/model_states', ModelStates, handle_model_state)
    rospy.spin()