#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
//BG SUBTRACTION
#include <opencv2/video/background_segm.hpp>

using namespace std;
using namespace cv;

// --------------------------------------------------------------- //
// ----------------------- Declarations -------------------------- //
// --------------------------------------------------------------- //
cv::Mat curr_frame_rgb, curr_frame_gray;
std::vector<bool> flag_vec(2);
const bool webcam_data_stream = false;

// Random color generator
cv::RNG rng(12345);

// Face detector
cv::CascadeClassifier face_cascade;
std::vector<cv::Rect> faces_vector;
string path_to_xml_face = "/home/istin/Documenti/1_CATKIN_SRCS/HRI_srcs/tracker_hri/misc/haarcascades/haarcascade_frontalface_alt.xml";

// Background subtractor
cv::Ptr<cv::BackgroundSubtractor> pMog2_sub;
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::Mat foreg_mask;


void rgbTrackerCB(const sensor_msgs::ImageConstPtr& msg);

// --------------------------------------------------------------- //
// ------------------------- Functions --------------------------- //
// --------------------------------------------------------------- //
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "tracker");
	ros::NodeHandle n;

	pMog2_sub = new cv::BackgroundSubtractorMOG();

	std::string topic;
	if (webcam_data_stream)
		topic = "/image_raw"; // WebCam
	else
		topic = "/camera/rgb/image_color"; // BAG
	
	ros::Subscriber rgb_sub = n.subscribe(topic, 1, rgbTrackerCB);

	ros::spin();
	return 0;
}

void rgbTrackerCB(const sensor_msgs::ImageConstPtr& msg)
{
	// Convert from sensor_msg to cv mat
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	curr_frame_rgb = cv_ptr->image.clone();

	if (webcam_data_stream)
	{
		cv::Point2f src_center(curr_frame_rgb.cols/2.0f, curr_frame_rgb.rows/2.0f);
		cv::Mat rot_mat = getRotationMatrix2D(src_center, 180.0f, 1.0);
		warpAffine(curr_frame_rgb, curr_frame_rgb, rot_mat, curr_frame_rgb.size());
	}

	// Background subtractor
	pMog2_sub->operator()(curr_frame_rgb, foreg_mask);

	cv::namedWindow("Foreground mask");
	cv::imshow("Foreground mask", foreg_mask);
	cv::waitKey(15);


	cv::Mat cont_mask = foreg_mask.clone();
	findContours(cont_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	if (contours.size() > 0)
	{
		std::vector<std::vector<cv::Point> > contours_poly_vec(contours.size());
		std::vector<cv::Rect> bound_box_vec(contours.size());

		for (int i = 0; i < contours.size(); i++)
		{
			std::vector<cv::Point>& cont_ = contours[i];
			float area = fabs(cv::contourArea(cont_));
			
			//! How much big must be the detection
			if (area > 10000)
			{
				cv::approxPolyDP(cont_, contours_poly_vec[i], 3, true);
				bound_box_vec[i] = cv::boundingRect(contours_poly_vec[i]);
				cv::drawContours(cont_mask, contours, i, cv::Scalar(80, 180, 80), CV_FILLED, 8, hierarchy);

				//! How much closer must be the subject to the robot (WARNING SIGN)
				if (bound_box_vec[i].tl().y < 220)
				{
					cv::rectangle(curr_frame_rgb, bound_box_vec[i].tl(), bound_box_vec[i].br(), cv::Scalar(10,180,50), 6, 8, 0);
					flag_vec[0] = true;
				}
			}
		}
	}

	cv::namedWindow("Contours", CV_WINDOW_NORMAL);
	cv::imshow("Contours", cont_mask);
	cv::waitKey(16);


	// Face detector
	//! Load the xml file into the cascade classifier
	bool out = face_cascade.load(path_to_xml_face);
	if (!out)
	{
		cerr << "ERROR Haar face filter!" << endl;
		return;
	}
	cv::cvtColor(curr_frame_rgb, curr_frame_gray, COLOR_BGR2GRAY);
	cv::equalizeHist(curr_frame_gray, curr_frame_gray);


	face_cascade.detectMultiScale(curr_frame_gray, faces_vector, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
	if (faces_vector.size() > 0)
	{
		flag_vec[1] = true;
		for (int i = 0; i < faces_vector.size(); i++)
		{
			cv::Point face_center(faces_vector[i].x + faces_vector[i].width*0.5, 
				faces_vector[i].y + faces_vector[i].height*0.5);

			cv::ellipse(curr_frame_rgb, face_center, Size( faces_vector[i].width*0.5, faces_vector[i].height*0.5), 
				0, 0, 360, Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), 4, 8, 0 );

			cv::Mat face_mask = curr_frame_gray(faces_vector[i]);
		}
	}

	if (flag_vec[0] || flag_vec[1])
	{
		cerr << "Publishing condition" << endl;
		// system("rostopic pub /diago/PNPConditionEvent std_msgs/String \"data: \'pDetected\'\" --once");
	}

	cv::namedWindow("RGB Datastream", CV_WINDOW_NORMAL);
	cv::imshow("RGB Datastream", curr_frame_rgb);
	cv::waitKey(16);
}