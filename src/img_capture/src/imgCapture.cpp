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
	int fps = 60;
	ros::Rate loop_rate(fps);
	
	//msg to send
	img_capture::imgRawData msg;

	//open camera
	Mat image;
	VideoCapture camera;
	camera.open(1);
	while(!camera.isOpened());	
	camera.set(CAP_PROP_FPS, fps);
	
	//sequence count
	uint seq_count = 0;

	//compress function
	int quality = 1;
	vector<int> param = vector<int>(2);
	//param[0] = CV_IMWRITE_PNG_COMPRESSION;
	//param[1] = quality;
	param[0] = CV_IMWRITE_JPEG_QUALITY;
	param[1] = 50;	

	while(ros::ok())
	{		
		//capture image
		camera >> image;
		msg.img.clear();
		
		//uncompressed method
		/*
		int size = image.rows * image.cols * image.channels();
		for(int i=0; i<size; ++i)
		{
			msg.img.push_back(image.data[i]);
		}
		*/

		//compressed method
		//imencode(".png", image, msg.img, param);
		imencode(".jpeg", image, msg.img, param);
		cout << image.rows * image.cols * image.channels() << "   " << msg.img.size() << endl;
		//fill msg information
		msg.header.seq = seq_count++;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "Test camera";
		msg.row = image.rows;
		msg.col = image.cols;		
		msg.channels = image.channels();
		msg.compressionFlag = CV_IMWRITE_PNG_COMPRESSION;
		msg.compressionQuality = quality;
		msg.compressName = ".png";

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
