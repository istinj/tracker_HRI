#pragma once
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <algorithm>
#include <cmath>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
//#include <laser_analysis/LaserObstacle.h>
//#include <laser_analysis/LaserObstacleMap.h>
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

	void laserscanCB(const sensor_msgs::LaserScanConstPtr& msg);
	void odomCB(const nav_msgs::OdometryConstPtr& msg);
	void depthCB(const sensor_msgs::ImageConstPtr& msg);
	void rgbCB(const sensor_msgs::ImageConstPtr& msg);
	//! Placeholders
//	void laserObsCB(const laser_analysis::LaserObstacleConstPtr& msg);
//	void laserMapCB(const laser_analysis::LaserObstacleMapConstPtr& msg);
	void getRobotPose(void);

	inline Eigen::Vector2f getLSRanges(void){return Eigen::Vector2f(_range_left, _range_right);}

private:
	tf::TransformListener *_listener;

	Eigen::Vector3f _obstacle_left;
	Eigen::Vector3f _obstacle_right;
	Eigen::Vector3f _diago_pose;

	float _range_left;
	float _range_right;

	bool _obstacle;

};
