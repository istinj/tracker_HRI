#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
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

#define FPS 1000/30
using namespace std;
using namespace cv;

// --------------------------------------------------------------- //
// ----------------------- Declarations -------------------------- //
// --------------------------------------------------------------- //
const bool webcam_data_stream = false;

cv::Mat curr_frame_rgb, curr_frame_gray;
std::vector<bool> flag_vec(2);

// Random color generator
cv::RNG rng(12345);

// Face detector
cv::CascadeClassifier face_cascade, body_cascade;
std::vector<cv::Rect> faces_vector, body_vector, body_vector_depth;
std::string path_to_xml_face = "/home/istin/Documenti/1_CATKIN_SRCS/HRI_srcs/tracker_hri/misc/haarcascades/haarcascade_frontalface_alt.xml";
std::string path_to_xml_body = "/home/istin/Documenti/1_CATKIN_SRCS/HRI_srcs/tracker_hri/misc/haarcascades/haarcascade_upperbody.xml";

// Background subtractor
cv::Ptr<cv::BackgroundSubtractor> pMog2_sub;
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::Mat foreg_mask;

// HOG
std::vector<float> b_detector, b_detector_depth;
cv::HOGDescriptor hog_descriptor, hog_descriptor_depth;

// Callbacks
void rgbTrackerCB(const sensor_msgs::ImageConstPtr& msg);
void depthTrackerCB(const sensor_msgs::ImageConstPtr& msg);
// Utilities
void display_image(const cv::Mat& image_, const std::string name_);

// --------------------------------------------------------------- //
// ------------------------- Functions --------------------------- //
// --------------------------------------------------------------- //
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "tracker");
	ros::NodeHandle n;

	pMog2_sub = new cv::BackgroundSubtractorMOG();
	b_detector = cv::HOGDescriptor::getDefaultPeopleDetector();
	hog_descriptor.setSVMDetector(b_detector);

	b_detector_depth = cv::HOGDescriptor::getDefaultPeopleDetector();
	hog_descriptor_depth.setSVMDetector(b_detector_depth);

	std::string topic_rgb(argv[1]);
	std::string topic_depth(argv[2]);
	
	ros::Subscriber rgb_sub = n.subscribe(topic_rgb, 1, rgbTrackerCB);
	ros::Subscriber depth_sub = n.subscribe(topic_depth, 1, depthTrackerCB);

	ros::spin();
	return 0;
}

void rgbTrackerCB(const sensor_msgs::ImageConstPtr& msg)
{
	// Convert from sensor_msg to cv mat
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	curr_frame_rgb = cv_ptr->image.clone();
	cv::cvtColor(curr_frame_rgb, curr_frame_gray, COLOR_BGR2GRAY);
	cv::equalizeHist(curr_frame_gray, curr_frame_gray);

	if (webcam_data_stream)
	{
		cv::Point2f src_center(curr_frame_rgb.cols/2.0f, curr_frame_rgb.rows/2.0f);
		cv::Mat rot_mat = getRotationMatrix2D(src_center, 180.0f, 1.0);
		warpAffine(curr_frame_rgb, curr_frame_rgb, rot_mat, curr_frame_rgb.size());
	}

//	// ********************************************* //
//	// *********** Background subtractor *********** //
//	// ********************************************* //
//	pMog2_sub->operator()(curr_frame_rgb, foreg_mask);
//	display_image(foreg_mask, "Foreground mask");
//
//
//	cv::Mat cont_mask = foreg_mask.clone();
//	findContours(cont_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//	if (contours.size() > 0)
//	{
//		std::vector<std::vector<cv::Point> > contours_poly_vec(contours.size());
//		std::vector<cv::Rect> bound_box_vec(contours.size());
//
//		for (int i = 0; i < contours.size(); i++)
//		{
//			std::vector<cv::Point>& cont_ = contours[i];
//			float area = fabs(cv::contourArea(cont_));
//			//! How much big must be the detection
//			if (area > 9000 && area < 35000)
//			{
//				cv::approxPolyDP(cont_, contours_poly_vec[i], 3, true);
//				bound_box_vec[i] = cv::boundingRect(contours_poly_vec[i]);
//				cv::drawContours(cont_mask, contours, i, cv::Scalar(80, 180, 80), CV_FILLED, 8, hierarchy);
//				cv::rectangle(curr_frame_rgb, bound_box_vec[i].tl(), bound_box_vec[i].br(), cv::Scalar(10,180,50), 6, 8, 0);
//
//				//! How much closer must be the subject to the robot (WARNING SIGN)
//				if (bound_box_vec[i].tl().y < 170)
//				{
//					cv::rectangle(curr_frame_rgb, bound_box_vec[i].tl(), bound_box_vec[i].br(), cv::Scalar(220,50,50), 6, 8, 0);
//					flag_vec[0] = true;
//				}
//			}
//		}
//	}
//
//	display_image(cont_mask, "Contours");
//
//	// ********************************************* //
//	// *************** Face detector *************** //
//	// ********************************************* //
//	bool out = face_cascade.load(path_to_xml_face);
//	if (!out)
//	{
//		cerr << "ERROR Haar face filter!" << endl;
//		return;
//	}
//
//	face_cascade.detectMultiScale(curr_frame_gray, faces_vector, 1.5, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
//	if (faces_vector.size() > 0)
//	{
//		flag_vec[1] = true;
//		for (int i = 0; i < faces_vector.size(); i++)
//		{
//			cv::Point face_center(faces_vector[i].x + faces_vector[i].width*0.5,
//				faces_vector[i].y + faces_vector[i].height*0.5);
//
//			cv::ellipse(curr_frame_rgb, face_center, Size( faces_vector[i].width*0.5, faces_vector[i].height*0.5),
//				0, 0, 360, Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), 4, 8, 0 );
//
//			cv::Mat face_mask = curr_frame_gray(faces_vector[i]);
//		}
//	}
//
//	// // ********************************************* //
//	// // ****************** HOG BODY ***************** //
//	// // ********************************************* //
//	// hog_descriptor.detectMultiScale(curr_frame_gray, body_vector, 0,
//	// 	cv::Size(10,10), cv::Size(40,40), 1.05, 1.3, false);
//	hog_descriptor.detectMultiScale(curr_frame_gray, body_vector, 0.3,
//		cv::Size(8,8), cv::Size(32, 32), 1.05, 2 );
//	for (int i = 0; i < body_vector.size(); i++)
//	{
//		cv::rectangle(curr_frame_gray, body_vector[i].tl(),
//			body_vector[i].br(), cv::Scalar(180,10,10));
//	}
//
//	display_image(curr_frame_gray, "HOGDescriptor");
	display_image(curr_frame_rgb, "RGB Datastream");

	if (flag_vec[0] || flag_vec[1])
	{
		// cerr << "Publishing condition" << endl;
		// system("rostopic pub /diago/PNPConditionEvent std_msgs/String \"data: \'pDetected\'\" --once");
	}
}


void depthTrackerCB(const sensor_msgs::ImageConstPtr& msg) 
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


	// HOG PEOPLE DETECTOR
	// hog_descriptor_depth.detectMultiScale(depth_image, body_vector_depth, 0.3,
	// 	cv::Size(8,8), cv::Size(32, 32), 1.05, 2 );
	// if (body_vector_depth.size() > 0)
	// {
	// 	cerr << "Depth HOG" << endl;
	// }

	display_image(depth_image, "Depth Image");
}


void display_image(const cv::Mat& image_, const std::string name_)
{
	cv::namedWindow(name_, CV_WINDOW_NORMAL);
	cv::imshow(name_, image_);
	cv::waitKey(FPS);
}
