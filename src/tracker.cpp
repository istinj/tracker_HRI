#include "tracker.h"
using namespace std;


Tracker::Tracker()
{
	_diago_pose.setZero();
	_prev_meas.setZero();

	_K.setIdentity();

	_obstacle = new Obstacle();
	_ekf = new KalmanFilter();

	_listener = new tf::TransformListener();

	_hog_descriptor = new cv::HOGDescriptor();
	_people_detector = cv::HOGDescriptor::getDefaultPeopleDetector();
	_hog_descriptor->setSVMDetector(_people_detector);

	_human_width = 60; // pixels
}

void Tracker::depthCB(const sensor_msgs::ImageConstPtr& msg)
{
	bool show_images = true;
	float min_range = 0.0f;
	float max_range = 0.1f;
	double min, max;
	float mean_depth = _obstacle->getDistance();

	_roi_vector.clear();

	//! Project the mean point of the obstacle to crop the
	//! depth image on the u-axis
	Eigen::Vector2f projected_obs, temp_point;
	projected_obs = _obstacle->projectPos(_K);
	temp_point = projected_obs + Eigen::Vector2f(0, -50);

	// Depth-based crop (wrt laser_analysis data)
	cv_bridge::CvImagePtr depth_bridge;
	depth_bridge = cv_bridge::toCvCopy(msg, "32FC1");

	for(int i = 0; i < depth_bridge->image.rows; i++)
	{
		for(int j = 0; j < depth_bridge->image.cols; j++)
		{
			if(depth_bridge->image.at<float>(i,j) < mean_depth - 1000 ||
					depth_bridge->image.at<float>(i,j) > mean_depth + 500)
				depth_bridge->image.at<float>(i,j) = 0;
		}
	}

	// Y-crop (wrt obstacle)
	if(projected_obs.norm() > 0)
	{
		for(int i = 0; i < depth_bridge->image.rows; i++)
		{
			for(int j = 0; j < depth_bridge->image.cols; j++)
			{
				if(j < projected_obs.x() - _human_width || j > projected_obs.x() + _human_width)
					depth_bridge->image.at<float>(i,j) = 0;
			}
		}
	}

	// Conversion into a displayble image
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

	// Find ROIs based on their area
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
			if(area > 5000.0f)
			{
				cv::approxPolyDP(cv::Mat(contour), contours_poly[i], 3, true);
				bound_rect_vect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
				if(bound_rect_vect[i].width < bound_rect_vect[i].height)
				{
					_roi_vector.push_back(bound_rect_vect[i]);
					cv::rectangle(depth_image, bound_rect_vect[i].tl(), bound_rect_vect[i].br(), CV_RGB(168,25,37), 3, CV_AA, 0);
					cout << BOLDBLUE << "Human found" << RESET << endl;
				}
			}
		}
	}

	//! Only for debug
	cv::line(depth_image, cv::Point(projected_obs.x(),projected_obs.y()),
			cv::Point(temp_point.x(),temp_point.y()), cv::Scalar(255,19,19));

	// Display the final depth image
	if(show_images)
		displayImage(depth_image, "Depth Image");
}

void Tracker::depthCamInfoCB(const sensor_msgs::CameraInfoConstPtr& msg)
{
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
			_K(i,j) = msg->K[i*3 + j];
	}
}

void Tracker::rgbCB(const sensor_msgs::ImageConstPtr& msg)
{
	bool show_images = true;

	std::vector<cv::Rect> detected_rect_vector;
	std::vector<cv::Point> temp_vector;

	// Convert from sensor_msg to cv mat
	cv_bridge::CvImageConstPtr cv_ptr;
	cv::Mat curr_frame_rgb, curr_frame_gray;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	curr_frame_rgb = cv_ptr->image.clone();
	cv::cvtColor(curr_frame_rgb, curr_frame_gray, cv::COLOR_BGR2GRAY);
	cv::equalizeHist(curr_frame_gray, curr_frame_gray);


	//! Showing the ROIs found in different windows.
	if(_roi_vector.size() > 0)
	{
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
			// Highlight the ROI center with a RED DOT;
			cv::Point roi_center(_roi_vector[i].tl().x + (_roi_vector[i].width/2),_roi_vector[i].tl().y + (_roi_vector[i].height/2));
			cv::circle(curr_frame_rgb, roi_center, 10 ,cv::Scalar(37,25,168), -1);

			// Perform HOG on the current ROI to find people
			_hog_descriptor->detectMultiScale(curr_frame_gray(_roi_vector[i]), detected_rect_vector,
					0.0, cv::Size(8,8), cv::Size(32,32), 1.15, 2);
//			_hog_descriptor->detect(curr_frame_gray(_roi_vector[i]), temp_vector, 0.0);
//			cout << RED << temp_vector.size() << RESET << endl;
//			temp_vector.clear();

			// If HOG found something, highlight it in the current rgb-frame
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
		detected_rect_vector.clear();
	}

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
	//! TODO: use variance to cut depth images on the y axis. -> zero
	//! TODO: track _obstacle_pos with a KALMAN FILTER.
	getRobotPose();

	Eigen::Vector2f temp_meas(msg->mx, msg->my);
	Eigen::Matrix2f temp_meas_cov;
	temp_meas_cov.setIdentity();
	temp_meas_cov(0,0) = 0.7;
//	temp_meas_cov *= powf((float)msg->var,2.0f);

	if(temp_meas.norm() == 0)
	{
		temp_meas = _prev_meas;
		temp_meas_cov(0,0) /= 8.0f;
	}
	else
		_obstacle->setSeenFlag();


	if (_obstacle->getFlag())
	{
		_obstacle->setObservation(temp_meas, temp_meas_cov);

		if(_ekf->getHistorySize() == 0)
		{
			cout << RED << "init" << RESET << endl;
			Eigen::Matrix4f temp_1;
			temp_1.setIdentity();
			_obstacle->initObs(Eigen::Vector4f(msg->mx, msg->my, 0, 0),temp_1);
		//	_obstacle->initObs(Eigen::Vector4f(17.0f, 17.0f, 0, 0),temp_1);
			_ekf->oneStep(_obstacle);
		}
		else
			_ekf->oneStep(_obstacle);
	}

	_obstacle->evaluateDistance();
	_prev_meas = temp_meas;

	// Print out stuff
	if (false)
	{
		cout << BOLDCYAN 	<< "New State:" 		<< endl; _obstacle->printState();
		cout << BOLDMAGENTA << "msg          :\t" 	<< msg->mx << " " << msg->my << RESET << endl;
		cout << BOLDBLUE 	<< "temp_meas    :\t" 	<< temp_meas.x() << " " << temp_meas.y() << RESET<< endl;
		cout << BOLDYELLOW 	<< "Mean dist[mm]:\t" 	<< _obstacle->getDistance() << RESET << endl;
		cout << BOLDGREEN 	<< "msg->varaianc:\t" 	<< msg->var << RESET << endl;
	}
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
