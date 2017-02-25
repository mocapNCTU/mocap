//proper library to include
#include <iostream>
#include <queue>
#include <vector>
#include <thread>
#include <memory>
#include <string>
#include <ctype.h>

//include ros usage
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <opencv2/opencv.hpp>


//include apriltag messages
#include "apriltag/apriltags_all_msgs.hpp"

using namespace std;
using namespace cv;

//Ros part
ros::Publisher infoBundle_publisher;
vector<ros::Subscriber> info_subscriber;
ros::Subscriber in;
ros::NodeHandlePtr node;

//default value
const int CAM_NUMBER = 1;
const int NO_CAM_NUM = -1;

//used parameter (some need to be set up)
int cameraSyncNumber = NO_CAM_NUM;  //need use get parameter to setup
string INFO_PREFIX, DEV_PREFIX;  //need use get parameter to setup
string mynamePrefix;
int seatSuccessSend = 0;
int seatCreated = 0;

//simple id extract
int extractID(string str)
{ 
	int pos = 0;
	for(int i=0; i<str.length(); ++i)
	{
		if(isdigit(str[i]))
		{
			pos = i;
			break;
		}
		if((i == (str.length()-1)) && (isdigit(str[i]) == false))
		{
			ROS_WARN("failed to extract id, exit with error");
			exit(-1);
		}
	}
	return stoi(str.substr(pos));
}

typedef class SYNC_SEAT
{
private:
	img_capture::apriltagInfos** seats;
	int seqNum;
	int seatRemain;

public:
	//constructor
	SYNC_SEAT()
	{
		seatRemain = cameraSyncNumber;
		seqNum = -1;
		seats = new img_capture::apriltagInfos*[cameraSyncNumber];
		for(int i=0; i<cameraSyncNumber; ++i)
		{
			seats[i] = nullptr;
		}
	}
	//destructor
	~SYNC_SEAT()
	{
		if(seats != nullptr)
		{
			this->release();
		}
	}

	//get seqNum
	int getSeqNum()
	{
		return seqNum;
	}

	void setSeqNum(int val)
	{
		this->seqNum = val;
	}

	//all signal fit successfully
	bool isSync()
	{
		return seatRemain == 0;	
	}

	//TODO should be called after "isSync()"
	img_capture::apriltagInfosBundle* genInfoBundle()
	{
		img_capture::apriltagInfosBundle* ret = new img_capture::apriltagInfosBundle();
		ret->seq = seqNum;
		for(int i=0; i<cameraSyncNumber; ++i)
		{
	 		ret->bundle.push_back(*seats[i]);
		}	
		return ret;
	}

	//smart genInfoBundle
	img_capture::apriltagInfosBundle* genInfoBundleSafe()
	{
		if(this->isSync())
		{
			return this->genInfoBundle();
		}
		
		return nullptr;
	}

	//sit in
	bool sitIn(const img_capture::apriltagInfos::ConstPtr& msg)
	{
		string str = msg->header.frame_id;
		int pos = 0;
		for(int i=0; i<msg->header.frame_id.length(); ++i)
		{
			if(isdigit(str[i]))
			{
				pos = i;
				break;
			}
			if((i == (msg->header.frame_id.length()-1)) && (isdigit(str[i]) == false))
			{
				ROS_WARN("cannot recognize camera id, exit with error");
				exit(-1);
			}
		}
		int id = stoi(str.substr(pos));
		if(id < cameraSyncNumber && seats[id] == nullptr)
		{
			seats[id] = new img_capture::apriltagInfos(*msg);
			--seatRemain;
			return true;
		}
		return false;
	}

	bool sitIn(img_capture::apriltagInfos* msg, int id)
	{
		if((id < cameraSyncNumber) && (seats[id] == nullptr))
		{
			seats[id] = new img_capture::apriltagInfos(*msg);
			--seatRemain;
			return true;
		}
		return false;
	}	
	
	img_capture::apriltagInfos** getData()
	{
		return seats;
	}

	//TODO : may error by using none correct delete
	void release()
	{
		for(int i=0; i<cameraSyncNumber; ++i)
		{
			delete seats[i];
		}
		delete[] seats;
	}

	void reset()
	{
		seatRemain = cameraSyncNumber;
		seqNum = -1;
		seats = new img_capture::apriltagInfos*[cameraSyncNumber];
		for(int i=0; i<cameraSyncNumber; ++i)
		{
			seats[i] = nullptr;
		}	
	}

	static SYNC_SEAT* create()
	{
		if(cameraSyncNumber != NO_CAM_NUM)
		{
			return new SYNC_SEAT();
		}
		else
		{
			return NULL;
		}
	}		
} Seats; 

bool isInCallBack = false;
Seats* syncMsg;
queue<Seats*> msgQueue;
queue<img_capture::apriltagInfos*>* msgArray;
 
//funcitons
void getParameters();
void apriltagInfos_rcv_callback(const img_capture::apriltagInfos::ConstPtr&);
void sync_publish();
void setupConnection();
void addSuccessCount()
{
	++seatSuccessSend;
}
void addSeatCreateNum()
{
	++seatCreated;
}
float successPercentage()
{
	return seatSuccessSend/seatCreated;
}

