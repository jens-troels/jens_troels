
#include <cstdlib>
#include <iostream>
#include "ros/ros.h"
#include <string>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "global_def.h"
#include <ctime>


int state;
int guiSig;
bool flag = true;

void callbackPackML(const std_msgs::Int32 &_state) {
    state = _state.data;
    flag = true;
}

void callbackGUI(const std_msgs::Int32 &_guiSig)
{
    guiSig = _guiSig.data;
}

int getState()
{
    return state;
}

void wait(double sec, int state)
{
    clock_t endwait;
    endwait = clock() + sec * CLOCKS_PER_SEC;
    while (clock() < endwait) {}
    ros::spinOnce();
    int states = getState();
    std::cout << state << std::endl;
    if(states == state)
        wait(sec, state);
}


int main(int argc,char* argv[]){

    ros::init(argc, argv, "PackML");

    ros::NodeHandle _n;
    //Pubs
    ros::Publisher statePub = _n.advertise<std_msgs::Int32>("packMLTopic", 1);
    ros::Publisher lightPub = _n.advertise<std_msgs::Int32>("lightTopic", 1);
    
    //Subs
    ros::Subscriber stateSub = _n.subscribe("packMLTopic", 1, &callbackPackML);
    ros::Subscriber gugiSub = _n.subscribe("guiTopic", 1, &callbackGUI);
    ros::Rate loop_rate(200);
    std::cout << "PackML started" << std::endl; 

    while(ros::ok()) {
        state = getState();
        std::cout << state << std::endl; 
        switch (state) {
            case STOPPED: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = RED_SOLID;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case RESETTING: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = YELLOW_FLASH;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case STARTING: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = GREEN_SOLID;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case IDLE: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = GREEN_FLASH;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case SUSPENDING: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = YELLOW_SOLID;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case SUSPENDED: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = YELLOW_SOLID;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case UNSUSPENDING: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = GREEN_SOLID;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case EXECUTE: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = GREEN_SOLID;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case STOPPING: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = RED_SOLID;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case ABORTING: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = RED_FLASH;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case ABORTED: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = RED_FLASH;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case CLEARING: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = RED_FLASH;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case HOLDING: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = GREEN_YELLOW_FLASH;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case HELD: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = GREEN_YELLOW_FLASH;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }

                break;
            }
            case UNHOLDING: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = GREEN_SOLID;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }
                break;
            }
            case COMPLETING: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = ALL_FLASH;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }
                break;
            }
            case COMPLETE: {
                if(flag)
                {
                    std_msgs::Int32 light;
                    light.data = ALL_FLASH;
                    lightPub.publish(light);
                    ros::spinOnce();
                    flag = false;
                }
                break;
            }
        }
        ros::spinOnce();
    }

    std::cout << "System has been shutdown!"<< std::endl;
    return 0;
}
