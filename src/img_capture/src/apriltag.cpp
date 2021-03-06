///include apriltag function
#include "apriltag/apriltags.h"
#include <omp.h>

string mynamePrefix;
int main(int argc, char **argv)
{
	//initial ros node
	ros::init(argc, argv, "apriltag_finder");
	node_ = boost::make_shared<ros::NodeHandle>();
	mynamePrefix = ros::this_node::getName();	
	mynamePrefix = mynamePrefix + "/";
	//check apriltag parameter
	GetParameterValues();
	InitializeTags();
	r = new ros::Rate(FPS);

	//setup ros connection with other nodes
	setupConnection(node_);
	
	//hold ros at this position
	//use single thread spin()
	//ros::spin();
	
	//use multi-thread spin()
	int THREAD_NUM = 4;
	ros::MultiThreadedSpinner spinner(THREAD_NUM);
	spinner.spin();

	return 0;
}

void setupConnection(ros::NodeHandlePtr node_obj)
{
	string subPath, pub_name;
	node_obj->param(mynamePrefix + "imgSource", subPath, subPath);
	node_obj->param(mynamePrefix + "out_name", pub_name, pub_name);
	if(subPath.empty() || pub_name.empty())
	{
		ROS_WARN("cannot find img source for apriltag, exit with error");
		exit(-1);
	}
	img_publisher = node_obj->advertise<img_capture::apriltagInfos>(mynamePrefix + pub_name, 1);
	img_subscriber = node_obj->subscribe(subPath + "/" + string("img_raw"), 1, img_rcv_callback);
}

void img_rcv_callback(const img_capture::imgRawData::ConstPtr& msg)
{
	//check if there is available camera information
	if(!has_camera_info_)
	{
		ROS_WARN("No Camera Info Received Yet");
		return ;
	}

	//extract image from msg
	/*
	Mat image = Mat(msg->row, msg->col, CV_8UC3);
	for(int i=0; i<msg->img.size(); ++i)
	{
		image.data[i] = msg->img[i];
	}
	*/
	Mat image = imdecode(Mat(msg->img), CV_LOAD_IMAGE_COLOR);	

	//turn image from BGR to gray image
	Mat subscribed_gray;
	cvtColor(image, subscribed_gray, cv::COLOR_RGB2GRAY);

    cv::Point2d opticalCenter;
    if ((camera_info_.K[2] > 1.0) && (camera_info_.K[5] > 1.0))
    {
        // cx,cy from intrinsic matric look reasonable, so we'll use that
        opticalCenter = cv::Point2d(camera_info_.K[5], camera_info_.K[2]);
    }
    else
    {
        opticalCenter = cv::Point2d(0.5 * subscribed_gray.rows, 0.5 * subscribed_gray.cols);
    }

    // Detect AprilTag markers in the image
	//TagDetectionArray is equal to std::vector<TagDetection> defined at file TagDetection.h by using typedef
    TagDetectionArray detections;

	//process detection
    detector_->process(subscribed_gray, opticalCenter, detections);

	//release image mat
	image.release();
	subscribed_gray.release();

	//fill header messages
    img_capture::apriltagInfos apriltag_detections = img_capture::apriltagInfos();
    apriltag_detections.header.frame_id = msg->header.frame_id;
    apriltag_detections.header.stamp = msg->header.stamp;
	apriltag_detections.header.seq = msg->header.seq;
	//ROS_INFO("AprilDetection of Seq : %d \n", msg->header.seq);
	//arrange tags which are detected into apriltagInfos

 
	for(unsigned int i = 0; i < detections.size(); ++i)
    {
        // skip bad detections
        if(!detections[i].good)
        {
            continue;
        }

        Eigen::Matrix4d pose;
        cv::Mat rvec;
        cv::Mat tvec;
        GetMarkerTransformUsingOpenCV(detections[i], pose, rvec, tvec);
        
        // Get this info from earlier code, don't extract it again
        Eigen::Matrix3d R = pose.block<3,3>(0,0);
        Eigen::Quaternion<double> q(R);
        
        double tag_size = GetTagSize(detections[i].id);
        //cout << tag_size << " " << detections[i].id << endl;
 
        // Fill in AprilTag detection.
        img_capture::apriltagInfo apriltag_det;
			//fill info's id and tag_size
        apriltag_det.id = detections[i].id;
        apriltag_det.tag_size = tag_size;
			//fill info's center point and its orientation
        apriltag_det.tagpose.point.xpos = pose(0, 3);
		apriltag_det.tagpose.point.ypos = pose(1, 3);
		apriltag_det.tagpose.point.zpos = pose(2, 3);
		apriltag_det.tagpose.orientation.xpos = q.x();
		apriltag_det.tagpose.orientation.ypos = q.y();
		apriltag_det.tagpose.orientation.zpos = q.z();
		apriltag_det.tagpose.orientation.wpos = q.w();
			//fill info's four corner points
		apriltag_det.corner.first.xpos = detections[i].p[0].x;
		apriltag_det.corner.first.ypos = detections[i].p[0].y; 
		apriltag_det.corner.first.zpos = 1;
		apriltag_det.corner.second.xpos = detections[i].p[1].x;
		apriltag_det.corner.second.ypos = detections[i].p[1].y;
		apriltag_det.corner.second.zpos = 1;
		apriltag_det.corner.third.xpos = detections[i].p[2].x;
		apriltag_det.corner.third.ypos = detections[i].p[2].y;
		apriltag_det.corner.third.zpos = 1;
		apriltag_det.corner.fourth.xpos = detections[i].p[3].x;
		apriltag_det.corner.fourth.ypos = detections[i].p[3].y;
		apriltag_det.corner.fourth.zpos = 1;
		apriltag_detections.tags.push_back(apriltag_det);
    }

	//publish msg
	img_publisher.publish(apriltag_detections);
	

	//release
	(apriltag_detections.tags).clear();
}

double GetTagSize(int tag_id)
{
    boost::unordered_map<size_t, double>::iterator tag_sizes_it = tag_sizes_.find(tag_id);
    if(tag_sizes_it != tag_sizes_.end()) {
        return tag_sizes_it->second;
    } else {
        return default_tag_size_;
    }
}

void GetMarkerTransformUsingOpenCV(const TagDetection& detection, Eigen::Matrix4d& transform, cv::Mat& rvec, cv::Mat& tvec)
{
    // Check if fx,fy or cx,cy are not set
    if ((camera_info_.K[0] == 0.0) || (camera_info_.K[4] == 0.0) || (camera_info_.K[2] == 0.0) || (camera_info_.K[5] == 0.0))
    {
        ROS_WARN("Warning: Camera intrinsic matrix K is not set, can't recover 3D pose");
    }

    double tag_size = GetTagSize(detection.id);

    std::vector<cv::Point3f> object_pts;
    std::vector<cv::Point2f> image_pts;
    double tag_radius = tag_size/2.;
    
    object_pts.push_back(cv::Point3f(-tag_radius, -tag_radius, 0));
    object_pts.push_back(cv::Point3f( tag_radius, -tag_radius, 0));
    object_pts.push_back(cv::Point3f( tag_radius,  tag_radius, 0));
    object_pts.push_back(cv::Point3f(-tag_radius,  tag_radius, 0));
    
    image_pts.push_back(detection.p[0]);
    image_pts.push_back(detection.p[1]);
    image_pts.push_back(detection.p[2]);
    image_pts.push_back(detection.p[3]);

    cv::Matx33f intrinsics(camera_info_.K[0], 0, camera_info_.K[2],
                           0, camera_info_.K[4], camera_info_.K[5],
                           0, 0, 1);
    
    cv::Vec4f distortion_coeff(camera_info_.D[0], camera_info_.D[1], camera_info_.D[2], camera_info_.D[3]);

    // Estimate 3D pose of tag
    // Methods:
    //   CV_ITERATIVE
    //     Iterative method based on Levenberg-Marquardt optimization.
    //     Finds the pose that minimizes reprojection error, being the sum of squared distances
    //     between the observed projections (image_points) and the projected points (object_pts).
    //   CV_P3P
    //     Based on: Gao et al, "Complete Solution Classification for the Perspective-Three-Point Problem"
    //     Requires exactly four object and image points.
    //   CV_EPNP
    //     Moreno-Noguer, Lepetit & Fua, "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    int method = CV_ITERATIVE;
    bool use_extrinsic_guess = false; // only used for ITERATIVE method
    cv::solvePnP(object_pts, image_pts, intrinsics, distortion_coeff, rvec, tvec, use_extrinsic_guess, method);

    cv::Matx33d r;
    cv::Rodrigues(rvec, r);
    Eigen::Matrix3d rot;
    rot << r(0,0), r(0,1), r(0,2),
           r(1,0), r(1,1), r(1,2),
           r(2,0), r(2,1), r(2,2);

    Eigen::Matrix4d T;
    T.topLeftCorner(3,3) = rot;
    T.col(3).head(3) <<
            tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    T.row(3) << 0,0,0,1;
    
    transform = T;
}

//TODO Callback for camera info
void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    camera_info_ = (*camera_info);
    has_camera_info_ = true;
}


void InitializeTags()
{
	//set up apriltag information
    tag_params.newQuadAlgorithm = 1;
    family_ = new TagFamily(tag_family_name_);
    detector_ = new TagDetector(*family_, tag_params);

	//read camera.yaml to generate cameraInfo
	std::string camera_name, camera_info_url;
	node_->param(mynamePrefix + "camera_name", camera_name, std::string("head_camera"));
	node_->param(mynamePrefix + "camera_info_url", camera_info_url, std::string(""));
	boost::shared_ptr<camera_info_manager::CameraInfoManager> camInfo;
	camInfo.reset(new camera_info_manager::CameraInfoManager(*node_, camera_name, camera_info_url));

	if(!camInfo->isCalibrated())
	{
		camInfo->setCameraName("fuck! I have no name");
		sensor_msgs::CameraInfo cam_info;
		cam_info.width = 640;
		cam_info.height = 480;
		camInfo->setCameraInfo(cam_info);
	}

	camera_info_ = sensor_msgs::CameraInfo(camInfo->getCameraInfo());
	has_camera_info_ = true;
}

void GetParameterValues()
{
	
    // Load node-wide configuration values.
    node_->param(mynamePrefix + "tag_family", tag_family_name_, DEFAULT_TAG_FAMILY);
    node_->param(mynamePrefix + "default_tag_size", default_tag_size_, DEFAULT_TAG_SIZE);
    node_->param(mynamePrefix + "display_type", display_type_, DEFAULT_DISPLAY_TYPE);
    node_->param(mynamePrefix + "marker_thickness", marker_thickness_, 0.01);

    node_->param(mynamePrefix + "viewer", viewer_, false);
    node_->param(mynamePrefix + "publish_detections_image", publish_detections_image_, false);
    node_->param(mynamePrefix + "display_marker_overlay", display_marker_overlay_, true);
    node_->param(mynamePrefix + "display_marker_outline", display_marker_outline_, false);
    node_->param(mynamePrefix + "display_marker_id", display_marker_id_, false);
    node_->param(mynamePrefix + "display_marker_edges", display_marker_edges_, false);
    node_->param(mynamePrefix + "display_marker_axes", display_marker_axes_, false);

    ROS_INFO("Tag Family: %s", tag_family_name_.c_str());
	
    // Load tag specific configuration values.
    XmlRpc::XmlRpcValue tag_data;
    node_->param(mynamePrefix + "tag_data", tag_data, tag_data);

    // Iterate through each tag in the configuration.
    XmlRpc::XmlRpcValue::ValueStruct::iterator it;
    for (it = tag_data.begin(); it != tag_data.end(); ++it)
    {
        // Retrieve the settings for the next tag.
        int tag_id = boost::lexical_cast<int>(it->first);
        XmlRpc::XmlRpcValue tag_values = it->second;

        // Load all the settings for this tag.
        if (tag_values.hasMember("size")) 
        {
            tag_sizes_[tag_id] = static_cast<double>(tag_values["size"]);
            ROS_INFO("Setting tag%d to size %f m.", tag_id, tag_sizes_[tag_id]);
        } 
		else 
		{
			ROS_INFO("Nothing useful");
		}
    }
}



