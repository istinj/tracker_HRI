#include "tracker.h"
using namespace std;


Tracker::Tracker()
{
	_obstacle = false;
	_diago_pose.setZero();
	_obstacle_pos.setZero();
	_mean_distance = 0;
	_prev_mean_distance = 0;

	_listener = new tf::TransformListener();

	_hog_descriptor = new cv::HOGDescriptor();
	_people_detector = cv::HOGDescriptor::getDefaultPeopleDetector();
	_hog_descriptor->setSVMDetector(_people_detector);
}

void Tracker::depthCB(const sensor_msgs::ImageConstPtr& msg)
{
	bool show_images = true;
	float min_range = 0.0f;
	float max_range = 0.1f;
	double min, max;

	_roi_vector.clear();

	cv_bridge::CvImagePtr depth_bridge;
	depth_bridge = cv_bridge::toCvCopy(msg, "32FC1");

	// mask - must substitute a dynamic mean_range value, gathered from laserscan (??)
//	float mean_depth = (float)1900.0;
	float mean_depth = _mean_distance;
	for(int i = 0; i < depth_bridge->image.rows; i++)
	{
		for(int j = 0; j < depth_bridge->image.cols; j++)
		{
			if(depth_bridge->image.at<float>(i,j) < mean_depth - 600 ||
					depth_bridge->image.at<float>(i,j) > mean_depth + 100)
				depth_bridge->image.at<float>(i,j) = 0;
		}
	}
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

	// Find center of the biggest blob
	std::vector<std::vector<cv::Point> > contour_vec;
	std::vector<cv::Vec4i> hierarchy;
	findContours(depth_image.clone(), contour_vec, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	if(contour_vec.size() > 0)
	{
		std::vector<std::vector<cv::Point> > contours_poly(contour_vec.size());
		std::vector<cv::Rect> boundRect_vec(contour_vec.size());

		for(int i = 0; i < contour_vec.size(); i++)
		{
			const std::vector<cv::Point>& contour = contour_vec[i];
			float area = fabs(cv::contourArea(cv::Mat(contour)));
			if(area > 20000.0f)
			{
				cv::approxPolyDP(cv::Mat(contour), contours_poly[i], 3, true);
				boundRect_vec[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
				_roi_vector.push_back(boundRect_vec[i]);
				cv::rectangle(depth_image, boundRect_vec[i].tl(), boundRect_vec[i].br(), CV_RGB(168,25,37), 3, CV_AA, 0);
			}
		}
	}
	cout << _roi_vector.size() << endl;

	if(show_images)
		displayImage(depth_image, "Depth Image");
}

void Tracker::rgbCB(const sensor_msgs::ImageConstPtr& msg)
{
	bool show_images = true;

	std::vector<cv::Rect> body_vector;
	std::vector<std::vector<cv::Point>> detected_point_vector;

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
	// Multiscale
//	_hog_descriptor->detectMultiScale(curr_frame_gray, body_vector, 0.3,
//		cv::Size(8,8), cv::Size(32, 32), 1.05, 2 );
//	for (int i = 0; i < body_vector.size(); i++)
//	{
//		cv::rectangle(curr_frame_gray, body_vector[i].tl(),
//			body_vector[i].br(), cv::Scalar(180,10,10));
//	}

	for(int i = 0; i < _roi_vector.size(); i++)
		_hog_descriptor->detect(curr_frame_gray(_roi_vector[i]), detected_point_vector[i]);
	//TODO: dentro il for fare il pushback dei detected_point_vector nello std::vector<std::vector<cv::Points>>

//	if(_roi_vector.size() > 0)
//	{
//		cout << "size " << _roi_vector.size() << endl;
//		for(int j = 0; j < _roi_vector.size(); j++)
//			cv::circle(curr_frame_gray, _roi_vector[j].tl(), 5 ,cv::Scalar(255,0,0));
//	}

	if(show_images)
	{
		displayImage(curr_frame_gray, "HOGDescriptor");
		displayImage(curr_frame_rgb, "RGB Datastream");
	}
	// cerr << "Publishing condition" << endl;
	// system("rostopic pub /diago/PNPConditionEvent std_msgs/String \"data: \'pDetected\'\" --once");
}

void Tracker::laserObsCB(const laser_analysis::LaserObstacleConstPtr& msg)
{
	return;
}

void Tracker::laserObsMapCB(const laser_analysis::LaserObstacleMapConstPtr& msg)
{
	_obstacle_pos << msg->mx, msg->my;
	cout << BOLDCYAN << "Mean point:  " << _obstacle_pos.x() << " " << _obstacle_pos.y() << RESET << endl;
	getRobotPose();
	cout << BOLDGREEN << "Diago pose:  " <<
			_diago_pose.x() << " " <<
			_diago_pose.y() << " " <<
			_diago_pose.z() << RESET << endl;

	Eigen::Vector2f temp;
	float mean_distance;
	temp = _obstacle_pos + _diago_pose.block<2,1>(0,0);

	mean_distance = sqrtf(powf(temp.x() - _diago_pose.x(), 2.0f) +
			powf(temp.y() - _diago_pose.y(), 2.0f));
	_mean_distance = mean_distance * 1000.0f;

	if(_mean_distance == 0.0f)
		_mean_distance = _prev_mean_distance;
	_prev_mean_distance = _mean_distance;

	cout << BOLDYELLOW << "Mean distance = " << mean_distance << RESET << endl;
	return;
}

void Tracker::getRobotPose(void)
{
	double roll, pitch, yaw;
	tf::StampedTransform T;

	_listener->waitForTransform("/map", "/diago/laser_frame", ros::Time(0), ros::Duration(2.0));
	_listener->lookupTransform("/map","/diago/laser_frame", ros::Time(0), T);

	tf::Quaternion q = T.getRotation();
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	_diago_pose << (float)T.getOrigin().x() ,
			(float)T.getOrigin().y(),
			(float)yaw;
	return;
}
