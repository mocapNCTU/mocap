#include "ros/ros.h"
#include "img_capture/imgRawData.h"
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	//initial this node and name it as ImgCapture
	ros::init(argc, argv, "ImgCapture");

	//create node handler
	ros::NodeHandle node_obj;

	//create publisher to sending message, topic name = "/img_raw", buffer number = 10(prevent the leakage caused by sending rate > recieve rate)
	ros::Publisher img_publisher = node_obj.advertise<img_capture::imgRawData>("/img_raw", 1000);

	//loop rate = 60hz
	int fps = 30;
	ros::Rate loop_rate(30);
	
	//msg to send
	img_capture::imgRawData msg;

	//open camera
	Mat image;
	VideoCapture camera;
	camera.open(1);
	while(!camera.isOpened());	
	camera.set(CAP_PROP_FPS, 30);
	
	//sequence count
	uint seq_count = 0;

	while(ros::ok())
	{		
		//capture image
		camera >> image;

		int size = image.rows * image.cols * image.channels();
		msg.img.clear();
		for(int i=0; i<size; ++i)
		{
			msg.img.push_back(image.data[i]);
		}

		//fill msg information
		msg.header.seq = seq_count++;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "Test camera";
		msg.row = image.rows;
		msg.col = image.cols;		
		msg.channels = image.channels();

		//sending image msg
		img_publisher.publish(msg);

		//use ros build timing control
		ros::spinOnce();
		loop_rate.sleep();
	}
	//exit, close the camera
	camera.release();
	return 0;
}
