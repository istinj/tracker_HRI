#include "tracker.h"
using namespace std;


Tracker::Tracker()
{
	_obstacle = false;
	_diago_pose.setZero();
	_obs_pos.setZero();
	_mean_distance = 0;
	_prev_mean_distance = 0;
	_obs_variance = 0;

	_listener = new tf::TransformListener();

	_hog_descriptor = new cv::HOGDescriptor();
	_people_detector = cv::HOGDescriptor::getDefaultPeopleDetector();
	_hog_descriptor->setSVMDetector(_people_detector);

	_haar_detector = new cv::CascadeClassifier();
	_haar_detector->load(_path_haar_upperbody);
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

	float mean_depth = _mean_distance;
	for(int i = 0; i < depth_bridge->image.rows; i++)
	{
		for(int j = 0; j < depth_bridge->image.cols; j++)
		{
			if(depth_bridge->image.at<float>(i,j) < mean_depth - 500 ||
					depth_bridge->image.at<float>(i,j) > mean_depth + 500)
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

	std::vector<std::vector<cv::Point> > contour_vec;
	std::vector<cv::Vec4i> hierarchy;
	findContours(depth_image.clone(), contour_vec, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	if(contour_vec.size() > 0)
	{
		std::vector<std::vector<cv::Point> > contours_poly(contour_vec.size());
		std::vector<cv::Rect> bound_rect_vect(contour_vec.size());

		for(int i = 0; i < contour_vec.size(); i++)
		{
			const std::vector<cv::Point>& contour = contour_vec[i];
			float area = fabs(cv::contourArea(cv::Mat(contour)));
			if(area > 8000.0f)
			{
				cv::approxPolyDP(cv::Mat(contour), contours_poly[i], 3, true);
				bound_rect_vect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
				if(bound_rect_vect[i].width < bound_rect_vect[i].height)
				{
					_roi_vector.push_back(bound_rect_vect[i]);
					cv::rectangle(depth_image, bound_rect_vect[i].tl(), bound_rect_vect[i].br(), CV_RGB(168,25,37), 3, CV_AA, 0);
				}
			}
		}
	}
	//else
		//_roi_vector.clear(); //! TODO: clear vector?
	if(show_images)
		displayImage(depth_image, "Depth Image");
}

void Tracker::rgbCB(const sensor_msgs::ImageConstPtr& msg)
{
	bool show_images = true;

	std::vector<cv::Rect> detected_rect_vector;
	std::vector<std::vector<cv::Rect>> detected_rect_super_vector;
	std::vector<std::vector<cv::Point>> detected_point_vector;
	std::vector<cv::Point> temp_vector;

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
//
//
	if(_roi_vector.size() > 0)
	{
		if(show_images)
		{
			for(int i = 0; i < _roi_vector.size(); i++)
			{
				std::string window_name = "rgb_" + std::to_string(i);
				displayImage(curr_frame_rgb(_roi_vector[i]), window_name);
			}
		}

//		//! How to make it work single scale?
//		for(int i = 0; i < _roi_vector.size(); i++)
//		{
//			_hog_descriptor->detect(curr_frame_gray(_roi_vector[i]), temp_vector, 0.0);
//			cout << RED << temp_vector.size() << RESET << endl;
//			detected_point_vector.push_back(temp_vector);
//			temp_vector.clear();
//		}


		for(int j = 0; j < _roi_vector.size(); j++)
		{
			cv::Point roi_center(_roi_vector[j].tl().x + (_roi_vector[j].width/2),_roi_vector[j].tl().y + (_roi_vector[j].height/2));
			cv::circle(curr_frame_rgb, roi_center, 10 ,cv::Scalar(37,25,168), -1);
		}

		for(int i = 0; i < _roi_vector.size(); i++)
		{
			_hog_descriptor->detectMultiScale(curr_frame_gray(_roi_vector[i]), detected_rect_vector,
					0.0, cv::Size(8,8), cv::Size(32,32), 1.15, 2);
			cout << "detected_rect_vector size: " << detected_rect_vector.size() << endl;
			detected_rect_super_vector.push_back(detected_rect_vector);

			if(detected_rect_vector.size() > 0)
			{
				for(int j = 0; j < detected_rect_vector.size(); j++)
				{
					cv::Point person_center(detected_rect_vector[j].tl().x + (detected_rect_vector[j].width/2),
							detected_rect_vector[j].tl().y + (detected_rect_vector[j].height/2));
					cv::circle(curr_frame_rgb, person_center, detected_rect_vector[j].height/4, cv::Scalar(37,168,25), 3);
				}
			}
			detected_rect_vector.clear();
		}
	}
	else
	{
		temp_vector.clear();
		detected_point_vector.clear();
		detected_rect_vector.clear();
		detected_rect_super_vector.clear();
	}

	/*
	// ********************************************* //
	// ************** HAAR UPPER-BODY ************** //
	// ********************************************* //
	if(_roi_vector.size() > 0)
	{
		//! Show ROIs - just for debug
		if(!show_images)
		{
			for(int i = 0; i < _roi_vector.size(); i++)
			{
				std::string window_name = "rgb_" + std::to_string(i);
				displayImage(curr_frame_rgb(_roi_vector[i]), window_name);
			}
		}
		for(int i = 0; i < _roi_vector.size(); i++)
		{
			//! Draw a circle in the center of the current ROI - red
			cv::Point roi_center(_roi_vector[i].tl().x + (_roi_vector[i].width/2),
					_roi_vector[i].tl().y + (_roi_vector[i].height/2));
			cv::circle(curr_frame_rgb, roi_center, 10 ,cv::Scalar(37,25,168), -1);

			//! Detect people using haar_upper_body detector in each ROI
			_haar_detector->detectMultiScale(curr_frame_gray(_roi_vector[i]), detected_rect_vector,
					1.2, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(16, 32));

			cout << "Haar detection size: " << detected_rect_vector.size() << endl;

			if(detected_rect_vector.size() > 0)
			{
				for(int j = 0; j < detected_rect_vector.size(); j++)
				{
					cv::Point person_center(detected_rect_vector[j].tl().x + (detected_rect_vector[j].width/2),
							detected_rect_vector[j].tl().y + (detected_rect_vector[j].height/2));
					cv::circle(curr_frame_rgb, person_center, detected_rect_vector[j].height/4, cv::Scalar(37,168,25), 3);
				}
			}
		}
	}/**/


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
	//! TODO: use variance to cut depth images on the y axis.
	//! TODO: track _obstacle_pos with a KALMAN FILTER.
	_obs_pos << msg->mx, msg->my;
	_obs_variance = msg->var;
	cout << BOLDCYAN << "Mean point:  " << _obs_pos.x() << " " << _obs_pos.y() << RESET << endl;
	cout << BOLDMAGENTA << "Variance:  " << _obs_variance << RESET << endl;
	getRobotPose();
	cout << BOLDGREEN << "Diago pose:  " <<
			_diago_pose.x() << " " <<
			_diago_pose.y() << " " <<
			_diago_pose.z() << RESET << endl;

	Eigen::Vector2f temp;
	float mean_distance;
	temp = _obs_pos + _diago_pose.block<2,1>(0,0);

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
