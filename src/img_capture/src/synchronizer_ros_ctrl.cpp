//include synchronizer function
#include "sync/sync_ros_ctrl.hpp"

int offset = 1;  //to avoid my laptop's webcam, use 0 if u don't have this problem
int main(int argc, char **argv)
{
	//initial ros node
	ros::init(argc, argv, "synchronizer_ros_ctrl");
	node = boost::make_shared<ros::NodeHandle>();
    r = new ros::Rate(FPS);
	
	//check sync parameters
	getParameters();
    	
	//setup ros connection with other nodes
	setupConnection();

	//ros time control
	ros::spin();

	return 0;
}

void getParameters()
{
	mynamePrefix = ros::this_node::getName() + "/";
	node->param(mynamePrefix + "cam_num", cameraSyncNumber, NO_CAM_NUM);
	node->param(mynamePrefix + "info_prefix", INFO_PREFIX, string(""));
	node->param(mynamePrefix + "device_prefix", DEV_PREFIX, string(""));
	node->param(mynamePrefix + "use_algorithm", useWhichAlgorithm, DEFAULT_ALGORITHM);
	if(useWhichAlgorithm == EXACT)
	{
		ROS_INFO("Use EXACT sync!");
	}
	else
	{
		ROS_INFO("Use APPROXIMATE sync!");
	}

	if((INFO_PREFIX.empty()) || (cameraSyncNumber == NO_CAM_NUM) || (DEV_PREFIX.empty()))
	{
		ROS_WARN("cannot fetch 'Synchronizer' node's parameters, exit with error");
		exit(-1);
	}
}

void rcv_call1(const img_capture::apriltagInfos::ConstPtr& a)
{
	Seats* msg = Seats::create();
	msg->sitIn(a, 0);
	infoBundle_publisher.publish(*(msg->genInfoBundle()));
	r->sleep();
	delete msg;
}

void rcv_call2(const INFO& a, const INFO& b)
{
	Seats* msg = Seats::create();
	msg->sitIn(a, 0);
	msg->sitIn(b, 1);
	infoBundle_publisher.publish(*(msg->genInfoBundle()));
	r->sleep();
	delete msg;
}

void rcv_call3(const INFO& a, const INFO& b, const INFO& c)
{
	Seats* msg = Seats::create();
	msg->sitIn(a, 0);
	msg->sitIn(b, 1);
	msg->sitIn(c, 2);
	infoBundle_publisher.publish(*(msg->genInfoBundle()));
	r->sleep();
	delete msg;
}

void rcv_call4(const INFO& a, const INFO& b, const INFO& c, const INFO& d)
{
	Seats* msg = Seats::create();
	msg->sitIn(a, 0);
	msg->sitIn(b, 1);
	msg->sitIn(c, 2);
	msg->sitIn(d, 3);
	infoBundle_publisher.publish(*(msg->genInfoBundle()));
	r->sleep();
	delete msg;
}

void rcv_call5(const INFO& a, const INFO& b, const INFO& c, const INFO& d, const INFO& e)
{
	Seats* msg = Seats::create();
	msg->sitIn(a, 0);
	msg->sitIn(b, 1);
	msg->sitIn(c, 2);
	msg->sitIn(d, 3);
	msg->sitIn(e, 4);
	infoBundle_publisher.publish(*(msg->genInfoBundle()));
	r->sleep();
	delete msg;
}

void rcv_call6(const INFO& a, const INFO& b, const INFO& c, const INFO& d, const INFO& e, const INFO& f)
{
	Seats* msg = Seats::create();
	msg->sitIn(a, 0);
	msg->sitIn(b, 1);
	msg->sitIn(c, 2);
	msg->sitIn(d, 3);
	msg->sitIn(e, 4);
	msg->sitIn(f, 5);
	infoBundle_publisher.publish(*(msg->genInfoBundle()));
	r->sleep();
	delete msg;
}

void rcv_call7(const INFO& a, const INFO& b, const INFO& c, const INFO& d, const INFO& e, const INFO& f, const INFO& g)
{
	Seats* msg = Seats::create();
	msg->sitIn(a, 0);
	msg->sitIn(b, 1);
	msg->sitIn(c, 2);
	msg->sitIn(d, 3);
	msg->sitIn(e, 4);
	msg->sitIn(f, 5);
	msg->sitIn(g, 6);
	infoBundle_publisher.publish(*(msg->genInfoBundle()));
	r->sleep();
	delete msg;
}

void rcv_call8(const INFO& a, const INFO& b, const INFO& c, const INFO& d, const INFO& e, const INFO& f, const INFO& g, const INFO& h)
{
	Seats* msg = Seats::create();
	msg->sitIn(a, 0);
	msg->sitIn(b, 1);
	msg->sitIn(c, 2);
	msg->sitIn(d, 3);
	msg->sitIn(e, 4);
	msg->sitIn(f, 5);
	msg->sitIn(g, 6);
	msg->sitIn(h, 7);
	infoBundle_publisher.publish(*(msg->genInfoBundle()));
	r->sleep();
	delete msg;
}

void rcv_call9(const INFO& a, const INFO& b, const INFO& c, const INFO& d, const INFO& e, const INFO& f, const INFO& g, const INFO& h, const INFO& i)
{
	Seats* msg = Seats::create();
	msg->sitIn(a, 0);
	msg->sitIn(b, 1);
	msg->sitIn(c, 2);
	msg->sitIn(d, 3);
	msg->sitIn(e, 4);
	msg->sitIn(f, 5);
	msg->sitIn(g, 6);
	msg->sitIn(h, 7);
	msg->sitIn(i, 8);
	infoBundle_publisher.publish(*(msg->genInfoBundle()));
	r->sleep();
	delete msg;
}


void setupConnection()
{
	//build up publisher
	infoBundle_publisher = node->advertise<img_capture::apriltagInfosBundle>("/apriltags_sync_info", 10);

	//build up subscribers
	if(cameraSyncNumber >= 2)
	{
		sub = new message_filters::Subscriber<AI>[cameraSyncNumber];
		for(int i=0; i<cameraSyncNumber; ++i)
		{
			sub[i].subscribe(*node, DEV_PREFIX + to_string(i+offset) + "/" + INFO_PREFIX, 10);
		}
	
		//setup subscribe sync
		switch(cameraSyncNumber)
		{
			case 1:
					break;
			case 2:
					if(!useWhichAlgorithm)
					{
						auto sync = new message_filters::Synchronizer<ESP2>(ESP2(SYNC_QUEUE_SIZE), sub[0], sub[1]);
						sync->registerCallback(boost::bind(&rcv_call2, _1, _2));
					}
					else
					{
						auto sync = new Synchronizer<ASP2>(ASP2(SYNC_QUEUE_SIZE), sub[0], sub[1]);
						sync->registerCallback(boost::bind(&rcv_call2, _1, _2));
					}
					break;
			case 3:
					if(!useWhichAlgorithm)
					{
						auto sync = new Synchronizer<ESP3>(ESP3(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2]);
						sync->registerCallback(boost::bind(&rcv_call3, _1, _2, _3));
					}
					else
					{
						auto sync = new Synchronizer<ASP3>(ASP3(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2]);
						sync->registerCallback(boost::bind(&rcv_call3, _1, _2, _3));
					}
					break;
			case 4:
					if(!useWhichAlgorithm)
					{
						auto sync = new Synchronizer<ESP4>(ESP4(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3]);
						sync->registerCallback(boost::bind(&rcv_call4, _1, _2, _3, _4));
					}
					else
					{
						auto sync = new Synchronizer<ASP4>(ASP4(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3]);
						sync->registerCallback(boost::bind(&rcv_call4, _1, _2, _3, _4));
					}
					break;
			case 5:
					if(!useWhichAlgorithm)
					{
						auto sync = new Synchronizer<ESP5>(ESP5(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3], sub[4]);
						sync->registerCallback(boost::bind(&rcv_call5, _1, _2, _3, _4, _5));
					}
					else
					{
						auto sync = new Synchronizer<ASP5>(ASP5(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3], sub[4]);
						sync->registerCallback(boost::bind(&rcv_call5, _1, _2, _3, _4, _5));
					}
					break;
			case 6:
					if(!useWhichAlgorithm)
					{
						auto sync = new Synchronizer<ESP6>(ESP6(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3], sub[4], sub[5]);
						sync->registerCallback(boost::bind(&rcv_call6, _1, _2, _3, _4, _5, _6));
					}
					else
					{
						auto sync = new Synchronizer<ASP6>(ASP6(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3], sub[4], sub[5]);
						sync->registerCallback(boost::bind(&rcv_call6, _1, _2, _3, _4, _5, _6));
					}
					break;
			case 7:
					if(!useWhichAlgorithm)
					{
						auto sync = new Synchronizer<ESP7>(ESP7(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6]);
						sync->registerCallback(boost::bind(&rcv_call7, _1, _2, _3, _4, _5, _6, _7));
					}
					else
					{
						auto sync = new Synchronizer<ASP7>(ASP7(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6]);
						sync->registerCallback(boost::bind(&rcv_call7, _1, _2, _3, _4, _5, _6, _7));
					}
					break;
			case 8:
					if(!useWhichAlgorithm)
					{
						auto sync = new Synchronizer<ESP8>(ESP8(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7]);
						sync->registerCallback(boost::bind(&rcv_call8, _1, _2, _3, _4, _5, _6, _7, _8));
					}
					else
					{
						auto sync = new Synchronizer<ASP8>(ASP8(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7]);
						sync->registerCallback(boost::bind(&rcv_call8, _1, _2, _3, _4, _5, _6, _7, _8));
					}
					break;
			case 9:
					if(!useWhichAlgorithm)
					{
						auto sync = new Synchronizer<ESP9>(ESP9(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7], sub[8]);
						sync->registerCallback(boost::bind(&rcv_call9, _1, _2, _3, _4, _5, _6, _7, _8, _9));
					}
					else
					{
						auto sync = new Synchronizer<ASP9>(ASP9(SYNC_QUEUE_SIZE), sub[0], sub[1], sub[2], sub[3], sub[4], sub[5], sub[6], sub[7], sub[8]);
						sync->registerCallback(boost::bind(&rcv_call9, _1, _2, _3, _4, _5, _6, _7, _8, _9));
					}
					break;
			default:
					ROS_WARN("Exceed max supported of the sync channel, exit with error");
		}
	}
	else
	{
		oneSub = node->subscribe(DEV_PREFIX + to_string(0+offset) + "/" + INFO_PREFIX, 10, rcv_call1);
	}
}
