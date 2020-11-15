
#include "ros/ros.h"
#include <string>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <rtde_receive_interface.h>
//#include "SubsAndPubs.h"

using namespace ur_rtde;

std::string ip = "192.168.0.201";

RTDEReceiveInterface receiver(ip);
bool flag;
bool receiveFlag;

void callbackGripper(const std_msgs::Bool _flag)
{
   //std::cout << "Disc" << std::endl; 
    flag = _flag.data;
}


int main(int argc,char* argv[]){

    ros::init(argc, argv, "RSD2019_REC");
    ros::NodeHandle _nhh;
    ros::Subscriber gripperSub = _nhh.subscribe("gripperTopic",1,&callbackGripper);
    ros::Publisher emPub = _nhh.advertise<std_msgs::Int32>("emTopic", 1000);
    ros::Publisher ackPub = _nhh.advertise<std_msgs::Bool>("ackTopic",1);

    ros::Rate loop_rate(10);
   	//int old = 0; 
   	//int neu = 0; 
    bool flagEm = true; 
    std::cout << "em_node started" << std::endl; 
    while(ros::ok())
    {
    	std::cout << receiver.getActualQ().at(0) << ", " << receiver.getActualQ().at(1) << ", "  << receiver.getActualQ().at(2) << ", " << receiver.getActualQ().at(3) << ", "  << receiver.getActualQ().at(4) << ", " << receiver.getActualQ().at(5) << std::endl;
    	sleep(3);
    	//std_msgs::Int32 em;
    	//em.data = 1;
    	//emPub.publish(em);
    	//std::cout << em << std::endl; 
    	int robotMode = receiver.getRobotMode();
    	//std::cout << robotMode << std::endl; 
    	if(robotMode == 3)
    	{ 
    		if(flagEm)
    		{
    			std_msgs::Int32 em;
    			em.data = 0;
    			emPub.publish(em);
    			sleep(2);
    			std::cout << em << std::endl; 
	    		flagEm = false; 
    		}
    		else
    		{
    			std_msgs::Int32 em1;
    			em1.data = 1;
    			emPub.publish(em1);
    			std::cout << em1 << std::endl; 
    		}

    	}	
    	if (robotMode == 7)
    	{ 
    		if(!flagEm)
    		{
		    	std_msgs::Int32 em2;
    			em2.data = 2;
    			emPub.publish(em2);
    			std::cout << em2 << std::endl; 
		    	flagEm = true;
	    	}

    	}

    	int boop = receiver.getActualDigitalOutputBits();
    	if(boop >= 10)
        {
    	    receiveFlag = true;
        } else
            receiveFlag = false;

    	if(receiveFlag == flag)
        {
    	    std_msgs::Bool pub;
    	    pub.data = true;
    	    ackPub.publish(pub);
    	    ros::spinOnce();
        } else{
            std_msgs::Bool pub;
            pub.data = false;
            ackPub.publish(pub);
            ros::spinOnce();
    	}



    	ros::spinOnce();
    	loop_rate.sleep();

    }

	return 0;
}