#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <cmath>
#include "colormod.h"
//ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
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

#define FPS (int)1000/30
using namespace std;
using namespace cv;

// --------------------------------------------------------------- //
// ----------------------- Declarations -------------------------- //
// --------------------------------------------------------------- //
cv::Mat curr_frame_rgb, curr_frame_gray;
bool show_images = true;
bool obstacle = true;

// Robot pose
tf::TransformListener *listener = NULL;
// Random color generator
cv::RNG rng(12345);

// Face detector
cv::CascadeClassifier face_cascade, body_cascade;
std::vector<cv::Rect> faces_vector, body_vector;
std::string path_to_xml_face = "/home/istin/Documenti/1_CATKIN_SRCS/HRI_srcs/tracker_hri/misc/haarcascades/haarcascade_frontalface_alt.xml";

// HOG
std::vector<float> b_detector;
cv::HOGDescriptor hog_descriptor;

// Callbacks
void rgbCB(const sensor_msgs::ImageConstPtr& msg);
void depthCB(const sensor_msgs::ImageConstPtr& msg);
void laserScanCB(const sensor_msgs::LaserScanConstPtr& msg);
void laserObsCB(const laser_analysis::LaserObstacleConstPtr& msg);
void laserMapCB(const laser_analysis::LaserObstacleMapConstPtr& msg);
// Utilities
void display_image(const cv::Mat& image_, const std::string name_);
void getRobotPose(Eigen::Vector3f& diago_pose_);
float roundPI2(float a);

// --------------------------------------------------------------- //
// ------------------------- Functions --------------------------- //
// --------------------------------------------------------------- //
int main(int argc, char *argv[])
{

	if (argc < 3)
	{
		cerr << BOLDYELLOW << "Too few arguments, you must specify two topics!" << RESET << endl;
		cerr << BOLDYELLOW << "Using default topics for RGB and DEPTH" << RESET << endl << endl;
		argv[1] = (char*)"/diago/top_camera/rgb/image_raw";
		argv[2] = (char*)"/diago/top_camera/depth/image_raw";
	}
	ros::init(argc, argv, "tracker");
	ros::NodeHandle n;

	b_detector = cv::HOGDescriptor::getDefaultPeopleDetector();
	hog_descriptor.setSVMDetector(b_detector);

	listener = new tf::TransformListener();

	/* BAG TOPICS
	 * /diago/cmd_vel -> geometry_msgs/Twist
	 * /diago/odom -> nav_msgs/Odometry
	 * /diago/scan -> sensor_msgs/LaserScan
	 * /diago/segway_status -> segway_rmpx/SegwayStatusStamped
	 * /diago/top_camera/depth/camera_info -> sensor_msgs/CameraInfo
	 * /diago/top_camera/depth/image_raw -> sensor_msgs/Image
	 * /diago/top_camera/rgb/camera_info -> sensor_msgs/CameraInfo
	 * /diago/top_camera/rgb/image_raw -> sensor_msgs/Image
	 *
	 *//**/

	std::string topic_rgb(argv[1]);
	std::string topic_depth(argv[2]);
	std::string topic_vel = "/diago/cmd_vel";
	std::string topic_laser_scan = "/diago/scan";
	std::string topic_laser_obs = "/diago/laser_obstacle";
	std::string topic_laser_map = "/diago/laser_obstacle_map";
	
//	ros::Subscriber rgb_sub = n.subscribe(topic_rgb, 1, rgbCB);
	ros::Subscriber depth_sub = n.subscribe(topic_depth, 1, depthCB);
	ros::Subscriber laser_scan_sub = n.subscribe(topic_laser_scan, 1, laserScanCB);
	ros::Subscriber laser_obs_sub = n.subscribe(topic_laser_obs, 1, laserObsCB);
	ros::Subscriber laser_map_sub = n.subscribe(topic_laser_map, 1, laserMapCB);

	ros::spin();
	return 0;
}


// CALLBACKS
void rgbCB(const sensor_msgs::ImageConstPtr& msg)
{
	// Convert from sensor_msg to cv mat
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	curr_frame_rgb = cv_ptr->image.clone();
	cv::cvtColor(curr_frame_rgb, curr_frame_gray, COLOR_BGR2GRAY);
	cv::equalizeHist(curr_frame_gray, curr_frame_gray);

	// ********************************************* //
	// ****************** HOG BODY ***************** //
	// ********************************************* //
//	hog_descriptor.detectMultiScale(curr_frame_gray, body_vector, 0.3,
//		cv::Size(8,8), cv::Size(32, 32), 1.05, 2 );
//	for (int i = 0; i < body_vector.size(); i++)
//	{
//		cv::rectangle(curr_frame_gray, body_vector[i].tl(),
//			body_vector[i].br(), cv::Scalar(180,10,10));
//	}
	if(show_images)
	{
		display_image(curr_frame_gray, "HOGDescriptor");
		display_image(curr_frame_rgb, "RGB Datastream");
	}
	// cerr << "Publishing condition" << endl;
	// system("rostopic pub /diago/PNPConditionEvent std_msgs/String \"data: \'pDetected\'\" --once");
}


void depthCB(const sensor_msgs::ImageConstPtr& msg) 
{
	// TOPIC /diago/top_camera/depth/image_raw
	float min_range = 0.0f;
	float max_range = 0.1f;
	double min, max;
	cv_bridge::CvImagePtr depth_bridge;
	depth_bridge = cv_bridge::toCvCopy(msg, "32FC1");


	cv::Mat depth_image(depth_bridge->image.rows,
		depth_bridge->image.cols,
		CV_8UC1);

	for (int i = 0; i < depth_bridge->image.rows; i++)
	{
		for (int j = 0; j < depth_bridge->image.cols; j++)
		{
			if (depth_bridge->image.ptr<float>(i)[j] > max_range)
			{
				max_range = depth_bridge->image.ptr<float>(i)[j];
			}
		}
	}

	for (int i = 0; i < depth_bridge->image.rows; i++)
	{
		// pointer to values -> modify the real values
		float* Di = depth_bridge->image.ptr<float>(i);
		char*  Ii = depth_image.ptr<char>(i);

		for (int j = 0; j < depth_bridge->image.cols; j++)
		{
			Ii[j] = (char) (255 * ((Di[j] - min_range)/(max_range - min_range)));
		}
	}
	if(show_images)
		display_image(depth_image, "Depth Image");
}


void laserScanCB(const sensor_msgs::LaserScanConstPtr& msg)
{
	int delta_angle = 4;
	bool first=true;
	float theta = msg->angle_min;
	float curr_range, x, y;
	float x_obs_right, y_obs_right, x_obs_left, y_obs_left;
	float detection_front_dist = 20.0f;
	float detection_side_dist = 0.5f;
	float rmin = detection_front_dist;

	Eigen::Vector2f obstacle_L, obstacle_R, temp;

	if(!obstacle)
		return;
	Eigen::Vector3f robot_pose;
	getRobotPose(robot_pose);


	float theta_0 = roundPI2((float)robot_pose(2)) - robot_pose(2); // rad
	theta -= theta_0;

	for(int i = 0; i < msg->ranges.size(); i += delta_angle)
	{
		if(msg->ranges[i] > msg->range_min)
		{
			curr_range = msg->ranges[i];
			temp << curr_range * cos(theta), curr_range * sin(theta);
			if(temp.x() > 0 &&
					temp.x() < detection_front_dist &&
					fabs(temp.y()) < detection_side_dist)
			{
				if(first)
				{
					obstacle_L = temp;
					first = false;
				}
				obstacle_R = temp;
			}
		}
		theta += msg->angle_increment * delta_angle;
	}
	cout << BOLDCYAN << "obstacle_L\n" << obstacle_L << endl;
	cout << "obstacle_R\n" << obstacle_R << RESET << endl << endl;
}

void laserObsCB(const laser_analysis::LaserObstacleConstPtr& msg)
{
//	cout << msg->y1 << "\t" << msg->y2 << endl;
}

void laserMapCB(const laser_analysis::LaserObstacleMapConstPtr& msg)
{
	Eigen::Vector2f laser_map(msg->mx, msg->my);
//	if(laser_map.norm() > 0)
//	{
////		cout << YELLOW << "Obstacle detected!" << endl;
//		obstacle = true;
//	}
//	else
//		obstacle = false;

}


// OTHER FUNCTIONS
void display_image(const cv::Mat& image_, const std::string name_)
{
	cv::namedWindow(name_, CV_WINDOW_NORMAL);
	cv::imshow(name_, image_);
	cv::waitKey(FPS);
}


void getRobotPose(Eigen::Vector3f& diago_pose_)
{
	double roll, pitch, yaw;
	tf::StampedTransform T;

	listener->waitForTransform("/map", "/diago/laser_frame", ros::Time(0), ros::Duration(1.0));
	listener->lookupTransform("/map","/diago/laser_frame", ros::Time(0), T);

	tf::Quaternion q = T.getRotation();
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	diago_pose_ << (float)T.getOrigin().x() ,
			(float)T.getOrigin().y(),
			(float)yaw;
	return;
}

float roundPI2(float a) // round angle to 0, PI/2, -PI/2, PI
{
	if ((a >= -M_PI_4 && a <= M_PI_4) || (a >= 7*M_PI_4 && a <= 2*M_PI))
		return 0;

	else if (a >= M_PI_4 && a <= 3*M_PI_4)
		return M_PI_2;

	else if ((a >= 3*M_PI_4 && a <= 5*M_PI_4) || (a >= -M_PI && a <= -3*M_PI_4))
		return M_PI;

	else if ((a >= 5*M_PI_4 && a <= 7*M_PI_4) || (a >= -3*M_PI_4 && a <= -M_PI_4))
		return -M_PI_2;

	else // should be not possible...
		return 0;
}
