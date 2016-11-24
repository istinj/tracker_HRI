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

// Random color generator
cv::RNG rng(12345);

// Face detector
cv::CascadeClassifier face_cascade;
std::vector<cv::Rect> faces_vector;
string path_to_xml_face = "/home/istin/Documenti/1_CATKIN_SRCS/HRI_srcs/tracker_hri/misc/haarcascades/haarcascade_frontalface_alt.xml";

// Background subtractor
cv::Ptr<cv::BackgroundSubtractor> pMog2_sub;
std::vector<cv::Point> contours;
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

	// BAG
	// ros::Subscriber rgb_sub = n.subscribe("/camera/rgb/image_color", 1, rgbTrackerCB);

	// WebCam
	pMog2_sub = new cv::BackgroundSubtractorMOG();
	ros::Subscriber rgb_sub = n.subscribe("/image_raw", 1, rgbTrackerCB);

	ros::spin();
	return 0;
}

void rgbTrackerCB(const sensor_msgs::ImageConstPtr& msg)
{
	// Convert from sensor_msg to cv mat
	bool webcam_data_stream = true;
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	curr_frame_rgb = cv_ptr->image.clone();

	if (webcam_data_stream)
	{
		cv::Point2f src_center(curr_frame_rgb.cols/2.0f, curr_frame_rgb.rows/2.0f);
		cv::Mat rot_mat = getRotationMatrix2D(src_center, 180.0f, 1.0);
		warpAffine(curr_frame_rgb, curr_frame_rgb, rot_mat, curr_frame_rgb.size());
	}

	//! Load the xml file into the cascade classifier
	bool out = face_cascade.load(path_to_xml_face);
	if (!out)
	{
		cerr << "ERROR Haar face filter!" << endl;
		return;
	}

	// Background subtractor
	pMog2_sub->operator()(curr_frame_rgb, foreg_mask);

	cv::namedWindow("Foreground mask");
	cv::imshow("Foreground mask", foreg_mask);
	cv::waitKey(15);






	// Face detector
	cv::cvtColor(curr_frame_rgb, curr_frame_gray, COLOR_BGR2GRAY);
	cv::equalizeHist(curr_frame_gray, curr_frame_gray);


	face_cascade.detectMultiScale(curr_frame_gray, faces_vector, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

	for (int i = 0; i < faces_vector.size(); i++)
	{
		cv::Point face_center(faces_vector[i].x + faces_vector[i].width*0.5, 
			faces_vector[i].y + faces_vector[i].height*0.5);

		cv::ellipse(curr_frame_rgb, face_center, Size( faces_vector[i].width*0.5, faces_vector[i].height*0.5), 
			0, 0, 360, Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), 4, 8, 0 );

		cv::Mat face_mask = curr_frame_gray(faces_vector[i]);
	}



	cv::namedWindow("RGB Datastream", CV_WINDOW_NORMAL);
	cv::imshow("RGB Datastream", curr_frame_rgb);
	cv::waitKey(16);
}