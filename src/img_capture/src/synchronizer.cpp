//include synchronizer function
#include "sync/sync.hpp"

int offset = 1; //to avoid my web cam
int main(int argc, char **argv)
{
	//initial ros node
	ros::init(argc, argv, "synchronizer");
	node = boost::make_shared<ros::NodeHandle>();
	
	//check apriltag parameter
	getParameters();
	syncMsg = Seats::create();
	msgArray = new queue<img_capture::apriltagInfos*>[cameraSyncNumber];
    	
	//setup ros connection with other nodes
	setupConnection();

	//hold ros at this position
	sync_publish();
	return 0;
}

void apriltagInfos_rcv_callback(const img_capture::apriltagInfos::ConstPtr& msg)
{
	int id = extractID(msg->header.frame_id) - offset;
	img_capture::apriltagInfos* info = new img_capture::apriltagInfos(*msg);
	msgArray[id].push(info);
}

void sync_publish()
{
	ros::Rate loop_rate(240);
	while(ros::ok())
	{
		//check if each has something to send
		bool needcheck = true;
		for(int i=0; i<cameraSyncNumber; ++i)
		{
			if(msgArray[i].empty())
			{
				needcheck = false;
				break;
			}		
		}

		if(needcheck)
		{
			//check if all in same sequence
			int seq = msgArray[0].front()->header.seq;
			int advSeq = seq;
			bool hasAdvance = false; //seqnumber > first seq
			bool hasDelay = false; //seqnumber < first seq
			for(int i=1; i<cameraSyncNumber; ++i)
			{
				int tmp = msgArray[i].front()->header.seq;
				if(tmp > seq)
				{
					hasAdvance = true;
					if(tmp > advSeq)
					{
						advSeq = tmp;
					}
				}
				else if(tmp < seq)
				{
					hasDelay = true;
				}
				if(hasAdvance && hasDelay)  //there must be lots of work to do after, so just break
				{
					break;
				}
			}
			
			if((!hasAdvance) && (!hasDelay))  //can send message
			{
				for(int i=0; i<cameraSyncNumber; ++i)
				{
					syncMsg->sitIn_shallow(msgArray[i].front(), i);
					msgArray[i].pop();
				}
				infoBundle_publisher.publish(*(syncMsg->genInfoBundle()));
				ros::spinOnce();
				syncMsg->reValid();
			}
			else if((!hasAdvance) && hasDelay)  //need to sync to "seq"
			{
				for(int i=0; i<cameraSyncNumber; ++i)  //let i from 1 will be okay too, since seq is get from i=0
				{
					if(msgArray[i].front()->header.seq < seq)
					{
						img_capture::apriltagInfos* info = msgArray[i].front();
						msgArray[i].pop();
						delete info;
					}
				}
				ros::spinOnce();
			}
			else  //need to sync to "seqAdv"
			{
				for(int i=0; i<cameraSyncNumber; ++i)
				{
					if(msgArray[i].front()->header.seq < advSeq)
					{
						img_capture::apriltagInfos* info = msgArray[i].front();
						msgArray[i].pop();
						delete info;
					}
				}
				ros::spinOnce();
			}
		}
		else
		{
			ros::spinOnce();
		}
		loop_rate.sleep();
	}
}

void setupConnection()
{
	//initial publisher
	infoBundle_publisher = node->advertise<img_capture::apriltagInfosBundle>("/apriltags_sync_info", 1000);
	
	//inital subscriber array
	for(int i=0; i<cameraSyncNumber; ++i)
	{
		//TODO
		info_subscriber.push_back(node -> subscribe(DEV_PREFIX + to_string(i+offset) + "/" + INFO_PREFIX, 1000, apriltagInfos_rcv_callback));
	}
}

void getParameters()
{
	mynamePrefix = ros::this_node::getName() + "/";
	node->param(mynamePrefix + "cam_num", cameraSyncNumber, NO_CAM_NUM);
	node->param(mynamePrefix + "info_prefix", INFO_PREFIX, string(""));
	node->param(mynamePrefix + "device_prefix", DEV_PREFIX, string(""));
	if((INFO_PREFIX.empty()) || (cameraSyncNumber == NO_CAM_NUM) || (DEV_PREFIX.empty()))
	{
		ROS_WARN("cannot fetch 'Synchronizer' node's parameters, exit with error");
		exit(-1);
	}
}
