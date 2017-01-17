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
	ros::init(argc, argv, "tracker");
	ros::NodeHandle n;

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
	 * OTHER TOPICS
	 * /localizer_ranges -> thin_navigation/LocalizerRanges
	 * /laser_obstacle -> laser_analysis/LaserObstacle
	 * /laser_obstacle_map -> laser_analysis/LaserObstacleMap
	 *//**/

	std::string topic_rgb = "/diago/top_camera/rgb/image_raw";
	std::string topic_depth = "/diago/top_camera/depth/image_raw";
	std::string topic_laser_scan = "/diago/scan";
	std::string topic_odom = "/diago/odom";
	std::string topic_laser_obs = "/diago/laser_obstacle";
	std::string topic_laser_obs_map = "/laser_obstacle_map";
	
	ros::Subscriber depth_sub = n.subscribe(topic_depth, 1, &Tracker::depthCB, &tracker);
	ros::Subscriber rgb_sub = n.subscribe(topic_rgb, 1, &Tracker::rgbCB, &tracker);
//	ros::Subscriber laser_obs_sub = n.subscribe(topic_laser_obs, 1, &Tracker::laserObsCB, &tracker);
	ros::Subscriber laser_obs_map_sub = n.subscribe(topic_laser_obs_map, 1, &Tracker::laserObsMapCB, &tracker);


	ros::spin();
	return 0;
}

