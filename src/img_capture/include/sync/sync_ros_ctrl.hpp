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
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

//include apriltag messages
#include "apriltag/apriltags_all_msgs.hpp"

using namespace std;
using namespace cv;
using namespace message_filters;

//Ros part
#define INFO img_capture::apriltagInfos::ConstPtr
#define AI img_capture::apriltagInfos

ros::Publisher infoBundle_publisher;
ros::Subscriber oneSub;
message_filters::Subscriber<img_capture::apriltagInfos>* sub;
ros::NodeHandlePtr node;
int FPS = 30;
ros::Rate *r;

//default value
const int CAM_NUMBER = 1;
const int NO_CAM_NUM = -1;
const bool APPROXIMATE = true;
const bool EXACT = false;
const bool DEFAULT_ALGORITHM = EXACT;

//used parameter (some need to be set up)
int cameraSyncNumber = NO_CAM_NUM;  //need use get parameter to setup
string INFO_PREFIX, DEV_PREFIX;  //need use get parameter to setup
string mynamePrefix;  //need use get parameter to setup
bool useWhichAlgorithm = DEFAULT_ALGORITHM;  //need use get parameter to setup

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

	bool sitIn_shallow(img_capture::apriltagInfos* msg, int id)
	{
		if(id < cameraSyncNumber)
		{
			seats[id] = msg;
			--seatRemain;
			return true;
		}
		return false;
	}	

	bool sitIn(const img_capture::apriltagInfos::ConstPtr& msg, int id)
	{
		if(id < cameraSyncNumber)
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

	void reValid()
	{
		for(int i=0; i<cameraSyncNumber; ++i)
		{
			delete seats[i];
			seats[i] = nullptr;
		}
		seatRemain = cameraSyncNumber;
		seqNum = -1;
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


//some definition
const int SYNC_QUEUE_SIZE = 10;

typedef sync_policies::ApproximateTime<AI, AI> ASP2;
typedef sync_policies::ApproximateTime<AI, AI, AI> ASP3;
typedef sync_policies::ApproximateTime<AI, AI, AI, AI> ASP4;
typedef sync_policies::ApproximateTime<AI, AI, AI, AI, AI> ASP5;
typedef sync_policies::ApproximateTime<AI, AI, AI, AI, AI, AI> ASP6;
typedef sync_policies::ApproximateTime<AI, AI, AI, AI, AI, AI, AI> ASP7;
typedef sync_policies::ApproximateTime<AI, AI, AI, AI, AI, AI, AI, AI> ASP8;
typedef sync_policies::ApproximateTime<AI, AI, AI, AI, AI, AI, AI, AI, AI> ASP9;

typedef sync_policies::ExactTime<AI, AI> ESP2;
typedef sync_policies::ExactTime<AI, AI, AI> ESP3;
typedef sync_policies::ExactTime<AI, AI, AI, AI> ESP4;
typedef sync_policies::ExactTime<AI, AI, AI, AI, AI> ESP5;
typedef sync_policies::ExactTime<AI, AI, AI, AI, AI, AI> ESP6;
typedef sync_policies::ExactTime<AI, AI, AI, AI, AI, AI, AI> ESP7;
typedef sync_policies::ExactTime<AI, AI, AI, AI, AI, AI, AI, AI> ESP8;
typedef sync_policies::ExactTime<AI, AI, AI, AI, AI, AI, AI, AI, AI> ESP9;

void rcv_call1(const INFO&);
void rcv_call2(const INFO&, const INFO&);
void rcv_call3(const INFO&, const INFO&, const INFO&);
void rcv_call4(const INFO&, const INFO&, const INFO&, const INFO&);
void rcv_call5(const INFO&, const INFO&, const INFO&, const INFO&, const INFO&);
void rcv_call6(const INFO&, const INFO&, const INFO&, const INFO&, const INFO&, const INFO&);
void rcv_call7(const INFO&, const INFO&, const INFO&, const INFO&, const INFO&, const INFO&, const INFO&);
void rcv_call8(const INFO&, const INFO&, const INFO&, const INFO&, const INFO&, const INFO&, const INFO&, const INFO&);
void rcv_call9(const INFO&, const INFO&, const INFO&, const INFO&, const INFO&, const INFO&, const INFO&, const INFO&, const INFO&);

void setupConnection();
void getParameters();















