//
// Created by jeh on 4/5/19.
//

#ifndef PROJECT_GRIPPER_H
#define PROJECT_GRIPPER_H

#include <ros/node_handle.h>
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Incr.h"
#include "wsg_50_common/Conf.h"
#include "ros/package.h"
#include "std_srvs/Empty.h"

using namespace std;


class Gripper {

public:
    Gripper();
    bool open(float _width);
    bool close(float _width);
    bool stop();
    bool home();
    bool setAcc(float _value);
    bool setSpeed(float _value);
    bool move(float _position, float _speed);
    bool grasp(float _position, float _speed);

private:
    ros::NodeHandle nh;
    //Service calls
    ros::ServiceClient clientMove = nh.serviceClient<wsg_50_common::Move>("wsg_50_driver/move");
    ros::ServiceClient clientMoveIncr = nh.serviceClient<wsg_50_common::Incr>("wsg_50_driver/move_incrementally");
    ros::ServiceClient clientHoming = nh.serviceClient<std_srvs::Empty>("wsg_50_driver/homing");
    ros::ServiceClient clientGrasp = nh.serviceClient<wsg_50_common::Move>("wsg_50_driver/grasp");
    ros::ServiceClient clientSetAccel = nh.serviceClient<wsg_50_common::Conf>("wsg_50_driver/set_acceleration");
    ros::ServiceClient clientSetSpeed = nh.serviceClient<wsg_50_common::Conf>("wsg_50_driver/set_speed");
    ros::ServiceClient clientStop = nh.serviceClient<std_srvs::Empty>("wsg_50_driver/stop");

    //Messages used by services
    wsg_50_common::Move moveSrv;
    wsg_50_common::Incr incrSrv;
    wsg_50_common::Conf confSrv;
    std_srvs::Empty emptySrv;

};


#endif //PROJECT_GRIPPER_H
