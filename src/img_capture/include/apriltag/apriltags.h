//proper library to include
#include <iostream>
#include <queue>

//include ros usage
#include "ros/ros.h"
#include <camera_info_manager/camera_info_manager.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <opencv2/opencv.hpp>

//include needed library to use
#include "TagDetector.h"
#include "TagDetection.h"
#include "TagFamily.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "yaml-cpp/yaml.h"
#include <sstream>
#include <fstream>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

//include apriltag messages
#include "apriltag/apriltags_all_msgs.hpp"

using namespace std;
using namespace cv;

//define april tag parameter
const double SMALL_TAG_SIZE = 0.0358968;
const double MED_TAG_SIZE = 0.06096;
const double PAGE_TAG_SIZE = 0.165;

const std::string DEFAULT_TAG_FAMILY = "Tag36h11";
const std::string DEFAULT_IMAGE_TOPIC = "image";
const std::string DEFAULT_CAMERA_INFO_TOPIC = "camera_info";
const std::string DEFAULT_MARKER_TOPIC = "marker_array";
const std::string DEFAULT_DETECTIONS_TOPIC = "detections";
const std::string DEFAULT_DETECTIONS_IMAGE_TOPIC = "detections_image";
const double DEFAULT_TAG_SIZE = MED_TAG_SIZE;
const std::string DEFAULT_DISPLAY_TYPE = "CUBE";

// AprilTag part
TagFamily* family_;
TagDetector* detector_;

TagDetectorParams tag_params;
std::string tag_data;
std::string tag_family_name_;
sensor_msgs::CameraInfo camera_info_;

// Ros part
int fps = 30;
ros::Publisher img_publisher;
ros::Subscriber img_subscriber;
ros::NodeHandlePtr node_;
ros::Rate r(fps);

// Settings and local information
bool viewer_;
bool publish_detections_image_;
double default_tag_size_;
double marker_thickness_;
boost::unordered_map<size_t, double> tag_sizes_;
bool running_;
bool has_camera_info_;
std::string display_type_;

bool display_marker_overlay_;
bool display_marker_outline_;
bool display_marker_id_;
bool display_marker_edges_;
bool display_marker_axes_;

//ex-author defined function
void GetMarkerTransformUsingOpenCV(const TagDetection& detection, Eigen::Matrix4d& transform, cv::Mat& rvec, cv::Mat& tvec);
void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);
void GetParameterValues();
double GetTagSize(int);
void InitializeTags(); //need to be called after node setup

//new defined function
	//rcv callback function include apriltag detection
void img_rcv_callback(const img_capture::imgRawData::ConstPtr&);
	//initialization function
void setupConnection(ros::NodeHandlePtr);
