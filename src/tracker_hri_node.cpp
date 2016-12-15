#include "utilities.h"
#include "tracker.h"

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

// --------------------------------------------------------------- //
// ------------------------- Functions --------------------------- //
// --------------------------------------------------------------- //
int main(int argc, char *argv[])
{

	if (argc < 3)
	{
		cerr << BOLDYELLOW << "If you want to change topic, write them as arguments." << endl;
		cerr << BOLDYELLOW << "Now using default topics for RGB and DEPTH" << RESET << endl << endl;
		argv[1] = (char*)"/diago/top_camera/rgb/image_raw";
		argv[2] = (char*)"/diago/top_camera/depth/image_raw";
		argv[3] = (char*)"/diago/scan";
	}
	ros::init(argc, argv, "tracker");
	ros::NodeHandle n;

	b_detector = cv::HOGDescriptor::getDefaultPeopleDetector();
	hog_descriptor.setSVMDetector(b_detector);

	listener = new tf::TransformListener();

	Tracker tracker;

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
	std::string topic_laser_scan(argv[3]);
	std::string topic_laser_obs = "/diago/laser_obstacle";
	std::string topic_laser_map = "/diago/laser_obstacle_map";
	
	ros::Subscriber rgb_sub = n.subscribe(topic_rgb, 1, &Tracker::rgbCB, &tracker);
	ros::Subscriber depth_sub = n.subscribe(topic_depth, 1, &Tracker::depthCB, &tracker);
	ros::Subscriber laser_scan_sub = n.subscribe(topic_laser_scan, 1, &Tracker::laserscanCB, &tracker);
//	ros::Subscriber laser_obs_sub = n.subscribe(topic_laser_obs, 1, laserObsCB);
//	ros::Subscriber laser_map_sub = n.subscribe(topic_laser_map, 1, laserMapCB);

	ros::spin();
	return 0;
}

