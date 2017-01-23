#pragma once
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <algorithm>
#include <cmath>
#include <string.h>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <laser_analysis/LaserObstacle.h>
#include <laser_analysis/LaserObstacleMap.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/background_segm.hpp>
//EIGEN
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "utilities.h"

class Tracker
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	Tracker(); // Ctor

	void depthCB(const sensor_msgs::ImageConstPtr& msg);
	void rgbCB(const sensor_msgs::ImageConstPtr& msg);
	void laserObsCB(const laser_analysis::LaserObstacleConstPtr& msg);
	void laserObsMapCB(const laser_analysis::LaserObstacleMapConstPtr& msg);
	void getRobotPose(void);

private:
	tf::TransformListener *_listener;
	cv::HOGDescriptor *_hog_descriptor;
	std::vector<float> _people_detector;
	std::vector<cv::Rect> _roi_vector;

	std::string _path_haar_upperbody = "/home/istin/Documenti/1_CATKIN_SRCS/HRI_srcs/tracker_hri/misc/haarcascades/haarcascade_upperbody.xml";
	cv::CascadeClassifier* _haar_detector;

	Eigen::Vector3f _diago_pose;
	Eigen::Vector2f _obstacle_pos;
	float _mean_distance, _prev_mean_distance;

	bool _obstacle;

};
