#include "tracker.h"
using namespace std;


Tracker::Tracker()
{
	_obstacle = false;
	_obstacle_left.setZero();
	_obstacle_right.setZero();
	_diago_pose.setZero();

	_range_left = 0;
	_range_right = 0;

	_listener = new tf::TransformListener();
}

void Tracker::laserscanCB(const sensor_msgs::LaserScanConstPtr& msg)
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
	getRobotPose();


	float theta_0 = roundPI2((float)_diago_pose(2)) - _diago_pose(2); // rad
	theta -= theta_0;
	int idx_L = 0, idx_R = 0;

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
					idx_L = i;
					first = false;
				}
				obstacle_R = temp;
				idx_R = i;
			}
		}
		theta += msg->angle_increment * delta_angle;
	}
//	cout << BOLDCYAN << "obstacle_L\n" << obstacle_L << endl;
//	cout << "obstacle_R\n" << obstacle_R << RESET << endl;
//	cout << idx_L << "\t" << idx_R << endl;
	cout << "ranges[idx_L] = " << msg->ranges[idx_L] << "\t"
			<< "ranges[idx_R] = " << msg->ranges[idx_R] << endl << endl;
}

void Tracker::depthCB(const sensor_msgs::ImageConstPtr& msg)
{
	bool show_images = true;
	float min_range = 0.0f;
	float max_range = 0.1f;
	double min, max;

	cv_bridge::CvImagePtr depth_bridge;
	depth_bridge = cv_bridge::toCvCopy(msg, "32FC1");

	// mask - must substitute a dynamic mean_range value, gathered from laserscan (??)
	// float mean_depth = (float)std::min(_range_left*1000,_range_right*1000);
	float mean_depth = (float)1600.0;
	for(int i = 0; i < depth_bridge->image.rows; i++)
	{
		for(int j = 0; j < depth_bridge->image.cols; j++)
		{
			if(depth_bridge->image.at<float>(i,j) < mean_depth - 600 ||
					depth_bridge->image.at<float>(i,j) > mean_depth + 100)
				depth_bridge->image.at<float>(i,j) = 0;
		}
	}
	/**/

	// --------------------------------------------------- //
	// -- CONVERTING THE RAW DATA INTO DISPLAYBLE IMAGE -- //
	// --------------------------------------------------- //
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
		displayImage(depth_image, "Depth Image");

	// --------------------------------------------------- //
	// -------------------- ANALISYS --------------------- //
	// --------------------------------------------------- //
}

void Tracker::rgbCB(const sensor_msgs::ImageConstPtr& msg)
{
	bool show_images = false;

	// Convert from sensor_msg to cv mat
	cv_bridge::CvImageConstPtr cv_ptr;
	cv::Mat curr_frame_rgb, curr_frame_gray;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	curr_frame_rgb = cv_ptr->image.clone();
	cv::cvtColor(curr_frame_rgb, curr_frame_gray, cv::COLOR_BGR2GRAY);
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
		displayImage(curr_frame_gray, "HOGDescriptor");
		displayImage(curr_frame_rgb, "RGB Datastream");
	}
	// cerr << "Publishing condition" << endl;
	// system("rostopic pub /diago/PNPConditionEvent std_msgs/String \"data: \'pDetected\'\" --once");
}


void Tracker::getRobotPose(void)
{
	double roll, pitch, yaw;
	tf::StampedTransform T;

	_listener->waitForTransform("/map", "/diago/laser_frame", ros::Time(0), ros::Duration(1.0));
	_listener->lookupTransform("/map","/diago/laser_frame", ros::Time(0), T);

	tf::Quaternion q = T.getRotation();
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	_diago_pose << (float)T.getOrigin().x() ,
			(float)T.getOrigin().y(),
			(float)yaw;
	return;
}
