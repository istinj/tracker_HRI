#include "utilities.h"

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


void displayImage(const cv::Mat& image_, const std::string name_)
{
	cv::namedWindow(name_, CV_WINDOW_NORMAL);
	cv::imshow(name_, image_);
	cv::waitKey(FPS);
}
