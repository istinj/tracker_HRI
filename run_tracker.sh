#!/bin/bash

NC='\033[0m'
BOLDRED='\033[1m\033[31m'
BOLDGREEN='\033[1m\033[32m'
BOLDYELLOW='\033[1m\033[33m'
BOLDBLUE='\033[1m\033[34m'
BOLDMAGENTA='\033[1m\033[35m'
BOLDCYAN='\033[1m\033[36m'
BOLDWHITE='\033[1m\033[37m'


#roscore
echo " "
echo -e "${BOLDRED}Starting roscore . . .${NC}"	
xterm -iconic -e 'roscore' &
sleep 3

# map server
echo -e "${BOLDBLUE}    Starting map_server with ${BOLDCYAN}DIS_first_floor${BOLDBLUE} map . . ."
xterm -iconic -e 'rosrun map_server map_server /home/istin/Documenti/1_CATKIN_SRCS/HRI_srcs/tracker_hri/maps/DIS_first_floor.yaml' &
sleep 3

# thin state pub
echo -e "    Starting thin_state_publisher . . ."
xterm -iconic -e 'rosrun thin_state_publisher thin_state_publisher_node -odom_topic /diago/odom -base_link_frame_id /diago/base_frame -odom_frame_id /diago/odom /home/istin/Documenti/1_CATKIN_SRCS/HRI_srcs/tracker_hri/misc/diago_transforms.txt' &
sleep 3

# thin_localizer
echo -e "    Starting thin_localizer_node . . ."
# xterm -iconic -e 'rosrun thin_navigation thin_localizer_node _odom_frame_id:=/diago/odom _base_frame_id:=/diago/base_frame _global_frame_id:=/map _laser_topic:=/diago/scan' & # no initial position -> localization
# xterm -iconic -e 'rosrun thin_navigation thin_localizer_node _odom_frame_id:=/diago/odom _base_frame_id:=/diago/base_frame _global_frame_id:=/map _laser_topic:=/diago/scan _initial_pose_x:=2 _initial_pose_y:=18 _initial_pose_a:=4.71' & # 1st gen bag
# xterm -iconic -e 'rosrun thin_navigation thin_localizer_node _odom_frame_id:=/diago/odom _base_frame_id:=/diago/base_frame _global_frame_id:=/map _laser_topic:=/diago/scan _initial_pose_x:=1.1 _initial_pose_y:=4.1 _initial_pose_a:=1.57' & # nav gen bag
# xterm -iconic -e 'rosrun thin_navigation thin_localizer_node _odom_frame_id:=/diago/odom _base_frame_id:=/diago/base_frame _global_frame_id:=/map _laser_topic:=/diago/scan _initial_pose_x:=12 _initial_pose_y:=1.5 _initial_pose_a:=3.14' & # gest gen bag
xterm -iconic -e 'rosrun thin_navigation thin_localizer_node _odom_frame_id:=/diago/odom _base_frame_id:=/diago/base_frame _global_frame_id:=/map _laser_topic:=/diago/scan _initial_pose_x:=9.1 _initial_pose_y:=1.5 _initial_pose_a:=0.0' & # gest gen bag
sleep 3

# rviz
if [ $# -eq 1 ]
	then
	echo -e "${BOLDBLUE}    Starting rviz . . ."
	xterm -iconic -e "rosrun rviz rviz -d diago.rviz" &
	sleep 3
else
	echo -e "${BOLDYELLOW}    Rviz not enabled"
fi

# laser obstacle
echo -e "${BOLDBLUE}    Starting laser_analysis . . ."
xterm -iconic -e 'roslaunch tracker_hri laser_analysis.launch' &
sleep 3

# tracker
echo -e "    Starting tracker_hri . . ."
xterm -e 'rosrun tracker_hri tracker_hri_node; read' &
sleep 3

# bag
echo -e "${BOLDGREEN}Launching the bag . . . ${NC}"
# xterm -e 'rosbag play ~/Documenti/0_ROS_Bags/both_diago.bag' &
# xterm -e 'rosbag play ~/Documenti/0_ROS_Bags/diago_nav_3_people_front_smooth_approach.bag' &

# xterm -e 'rosbag play ~/Documenti/0_ROS_Bags/diago_gest_robot_move.bag' &
xterm -e 'rosbag play ~/Documenti/0_ROS_Bags/diago_gest_person_move.bag' &
sleep 1
echo " "
