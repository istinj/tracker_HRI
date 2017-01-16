#!/bin/bash

# map server
xterm -e 'rosrun map_server map_server /home/istin/Documenti/1_CATKIN_SRCS/HRI_srcs/tracker_hri/maps/DIS_first_floor.yaml' &
sleep 3

# thin state pub
xterm -e 'rosrun thin_state_publisher thin_state_publisher_node -odom_topic /diago/odom -base_link_frame_id /diago/base_frame -odom_frame_id /diago/odom /home/istin/Documenti/1_CATKIN_SRCS/HRI_srcs/tracker_hri/misc/diago_transforms.txt' &
sleep 3

# thin_localizer
xterm -e 'rosrun thin_navigation thin_localizer_node _odom_frame_id:=/diago/odom _base_frame_id:=/diago/base_frame _global_frame_id:=/map _laser_topic:=/diago/scan' &
# xterm -e 'rosrun thin_navigation thin_localizer_node _odom_frame_id:=/diago/odom _base_frame_id:=/diago/base_frame _global_frame_id:=/map _laser_topic:=/diago/scan _initial_pose_x:=1.68 _initial_pose_y:=30.2 _initial_pose_a:=270' &
sleep 3

# rviz
xterm -e "rosrun rviz rviz -d diago.rviz" &
sleep 3

# # tracker
# xterm -e 'rosrun tracker_hri tracker_hri_node' &
# sleep 3

# bag
# xterm -e 'rosbag play ~/Documenti/0_ROS_Bags/both_diago.bag' &
xterm -e 'rosbag play ~/Documenti/0_ROS_Bags/both_diago.bag -l' &
sleep 1

