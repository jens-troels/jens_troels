//
// Created by jeh on 4/5/19.
//

#include "Gripper.h"

Gripper::Gripper()
{

}

bool Gripper::close(float _width) {

    incrSrv.request.direction = "close";
    incrSrv.request.increment = _width;

    if (clientMoveIncr.call(incrSrv))
    {
        cout << "Succesfully called the request" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
}

bool Gripper::open(float _width) {

    incrSrv.request.direction = "open";
    incrSrv.request.increment = _width;

    if (clientMoveIncr.call(incrSrv))
    {
        cout << "Succesfully called the request" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
}

bool Gripper::stop() {

    if (clientStop.call(emptySrv))
    {
        cout << "Succesfully called the request" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
}

bool Gripper::home() {

    if (clientHoming.call(emptySrv))
    {
        cout << "Succesfully called the request" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
}

bool Gripper::setAcc(float _value)
{

    confSrv.request.val = _value;

    if (clientSetAccel.call(confSrv))
    {
        cout << "Succesfully called the request" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;

}

bool Gripper::setSpeed(float _value)
{
    confSrv.request.val = _value;

    if (clientSetSpeed.call(confSrv))
    {
        cout << "Succesfully called the request" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
}

bool Gripper::move(float _position, float _speed)
{
    moveSrv.request.width = _position;
    moveSrv.request.speed = _speed;

    if (clientMove.call(moveSrv))
    {
        cout << "Succesfully called the request" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
}

bool Gripper::grasp(float _position, float _speed)
{
    moveSrv.request.width = _position;
    moveSrv.request.speed = _speed;

    if (clientGrasp.call(moveSrv))
    {
        cout << "Succesfully called the request" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
}