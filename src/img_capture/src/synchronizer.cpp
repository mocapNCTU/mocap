//include synchronizer function
#include "sync/sync.hpp"

int main(int argc, char **argv)
{
	//initial ros node
	ros::init(argc, argv, "synchronizer");
	node = boost::make_shared<ros::NodeHandle>();
	
	//check apriltag parameter
	getParameters();
	syncMsg = Seats::create();

	//setup ros connection with other nodes
	setupConnection();
	
	//hold ros at this position
	sync_publish();
	return 0;
}

void apriltagInfos_rcv_callback(const img_capture::apriltagInfos::ConstPtr& msg)
{
	cout << msg->header.frame_id  <<endl;
	int seq = msg->header.seq;
	int currentSeq = syncMsg->getSeqNum();
	if(currentSeq != -1)  //means there are some msgs queueing right now
	{
		if(seq == currentSeq) //same sequence	
		{
			syncMsg->sitIn(msg);
			if(syncMsg->isSync())
			{
				msgQueue.push(syncMsg);
			}
			syncMsg = Seats::create();
		}
		else
		{
			/*
				if seq > currentSeq
				    means currentSeq lose packet and there is no way to resend
				so we sync the new seq number
				else (seq < currentSeq)
				    never happen(it's TCP based by default), TODO if use UDP based
			*/
			if(seq > currentSeq)
			{
				syncMsg->release();
				syncMsg->reset();
				syncMsg->setSeqNum(seq);
				syncMsg->sitIn(msg);
				if(syncMsg->isSync())
				{
					msgQueue.push(syncMsg);
				}
				syncMsg = Seats::create();
			}
			else  
			{
				//never happen
				
			}
		}
	}
	else  //means there is no msgs queueing right now
	{
		syncMsg->setSeqNum(seq);
		syncMsg->sitIn(msg);
		if(syncMsg->isSync())
		{
			msgQueue.push(syncMsg);
		}
		syncMsg = Seats::create();
	}
}

void sync_publish()
{
	while(ros::ok())
	{
		if(!msgQueue.empty())
		{
			Seats* tmp = msgQueue.front();
			infoBundle_publisher.publish(*(tmp->genInfoBundle()));
			msgQueue.pop();
			delete tmp;
		}
		else
		{
			//do nothing
		}
		ros::spinOnce();
	}
}

void setupConnection()
{
	//initial publisher
	infoBundle_publisher = node->advertise<img_capture::apriltagInfosBundle>("/apriltags_sync_info", 1000);
	
	//inital subscriber array
	for(int i=0; i<cameraSyncNumber; ++i)
	{
		info_subscriber.push_back(node -> subscribe(DEV_PREFIX + to_string(i) + "/" + INFO_PREFIX, 1000, apriltagInfos_rcv_callback));
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
