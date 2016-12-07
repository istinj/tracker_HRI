#!/bin/bash

xterm -e 'roscore' &
sleep 2

#xterm -e '/camera/rgb/image_raw /camera/depth/image_raw' &
xterm -e 'rosrun tracker_hri tracker_hri_node /diago/top_camera/rgb/image_raw /diago/top_camera/depth/image_raw' &
sleep 1

xterm -e 'rosbag play ~/Documenti/0_ROS_Bags/both_diago_2015-12-11-16-32-36.bag' &
#xterm -e 'rosbag play ~/Documenti/0_ROS_Bags/robot_diago_2015-12-11-16-24-57.bag' &
#xterm -e 'rosbag play ~/Documenti/0_ROS_Bags/2015-11-17-15-00-45.bag' &
sleep 1

