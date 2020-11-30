#!/usr/bin/env python

import rosgraph
import roslaunch
from rosbag_record import RosbagRecord
import rospy

track_prefix = 'track'
track_number = 2

rosbag_command = 'rosbag record -a --duration=30 --lz4 -q'
record_dir = '/home/shenc/recorded'

def main():

    # launch = roslaunch.scriptapi.ROSLaunch()
    # launch.start()
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    for track_num in range(1, track_number+1):
        track_name = track_prefix + str(track_num)
        rospy.logdebug("starting {}".format(track_name))
        # args = 'world_name:={} gazebo_gui:=false rviz:=false'.format(track_name)
        
        spawn_map_args = ['worldmodel_interface','spawn_map_robot.launch','world_name:='+track_name, 'gazebo_gui:=false','rviz:=true']
        
        random_ctrl_args = ['worldmodel_interface', 'start_random_control.launch']
        
        spawn_map_launch_file = roslaunch.rlutil.resolve_launch_arguments(spawn_map_args)[0]
        spawn_map_launch_args = spawn_map_args[2:]

        random_ctrl_launch_file = roslaunch.rlutil.resolve_launch_arguments(random_ctrl_args)[0]

        roslaunch_files = [(spawn_map_launch_file, spawn_map_launch_args), (random_ctrl_launch_file)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files)

        launch.start()
        command = rosbag_command + ' -o {}'.format(track_name)
        RosbagRecord(command, record_dir)
        rospy.sleep(35)
        # rospy.spin()
        launch.shutdown()

        while rosgraph.is_master_online():
            rospy.logerr_throttle_identical("ROSMASTER still running")
    
    # launch.shutdown()



if __name__ == "__main__":
    main()