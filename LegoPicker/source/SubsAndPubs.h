//
// Created by jeh on 9/30/19.
//

#ifndef TESTPKG_SUBSANDPUBS_H
#define TESTPKG_SUBSANDPUBS_H


#include "ros/ros.h"
#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Bool.h"
#include "unistd.h"
#include "global_def.h"
#include <ctime>


class SubsAndPubs {
public:
    SubsAndPubs(int argc, char *argv[]);
    void commenceWork();
    void rosSpinOnce();
    void callbackPackML(const std_msgs::Int32& _state);
    void callbackSig(const std_msgs::Int32::ConstPtr& _signal);
    void callbackId(const std_msgs::Int32& _id);
    void callbackTicket(const std_msgs::String::ConstPtr& _ticket);
    void callbackPISig(const std_msgs::String::ConstPtr& _PIsignal);
    void callbackBricks(const std_msgs::String::ConstPtr& _bricks);
    void callbackMir(const std_msgs::Int32::ConstPtr &_mirSig);
    void callbackMirRelease(const std_msgs::Int32::ConstPtr &_mirReleaseSig);
    void callbackGUI(const std_msgs::Int32::ConstPtr &_guiSig);
    void callbackEm(const std_msgs::Int32& _emSig);
    void callbackAck(const std_msgs::Bool _ack);
    int getEmSig();
    int getMirSig();
    int getCurrentId();
    std::string getCurrentTicket();
    std::string getPISignal();
    int getMESSignal();
    int getCurrentSig();
    std::string getBrickVector();
    void setMESSignal(int _signal);
    void setId(int _id);
    void setTicket(std::string _ticket);
    void setColor(int _color);
    void setLight(int _light);
    void setGripper(bool flag);
    void topicSleep(double _sleepy);
    void setMirSignal(int _mirSignal);
    void setFakeMir(int _fakeMirSig);
    void waitMir(double sec, int mirSig_);
    void waitGugi(double sec, int gugiSig);
    void waitStart(double sec, int gugiSig);
    void waitAck(double sec, bool gripperFlag);
    int getState();
    bool getAck();
    void setState(int _state);
    int getGUISig();
    void setGUI(int _guiSig);


    bool waitFlag = false;

    ~SubsAndPubs();

private:

    std::string packMLTopic = "packMLTopic";
    std::string signalRoboTopic = "signalRoboTopic";
    std::string signalMESTopic = "signalMESTopic";
    std::string idTopic = "idTopic";
    std::string ticketTopic = "ticketTopic";
    std::string PISignalTopic = "PISignalTopic";
    std::string PIColorTopic = "PIColorTopic";
    std::string brickTopic = "brickTopic";
    std::string lightTopic = "lightTopic";
    std::string gripperTopic = "gripperTopic";
    std::string mirRoboTopic = "mirRoboTopic";
    std::string roboMirTopic = "roboMirTopic";
    std::string guiTopic = "guiTopic";
    std::string emTopic = "emTopic";
    std::string ackTopic = "ackTopic";

    //Pubs
    ros::Publisher signalRoboPub;
    ros::Publisher idPub;
    ros::Publisher ticketPub;
    ros::Publisher colorPub;
    ros::Publisher lightPub;
    ros::Publisher gripperPub; 
    ros::Publisher roboMirPub;
    ros::Publisher fakeMirPub;
    ros::Publisher packMLPub;
    ros::Publisher guiPub;
    ros::Publisher emPub; 


    //Subs
    ros::Subscriber signalMESSub;
    ros::Subscriber idSub;
    ros::Subscriber ticketSub;
    ros::Subscriber PISignalSub;
    ros::Subscriber brickSub;
    ros::Subscriber mirRoboSub; 
    ros::Subscriber packMLSub;
    ros::Subscriber guiSub;
    ros::Subscriber emSub; 
    ros::Subscriber ackSub;

    int currentId;
    int state;
    std::string currentTicket;
    std::string PISignal;
    int MESSignal;
    int mirSig; 
    std::string brickVector;
    double sleepy = 2000;
    int guiSig;
    int emSig;
    bool ackFlag;

};


#endif //TESTPKG_SUBSANDPUBS_H