//
// Created by jeh on 11/27/18.
//

#include "Man.h"


Man::Man()
{

}
Man::Man(Point _pos, Point _dir, float _turnCost, float _forwardCost, float _returnCanCost, float _getCanCost)
{
    pos = _pos;
    dir = _dir;
    turnCost = _turnCost;
    forwardCost = _forwardCost;
    returnCanCost = _returnCanCost;
    getCanCost = _getCanCost;
}
Point Man::getPos()
{
    return pos;
}
Point Man::getDir()
{
    return dir;
}
void Man::setPos(Point _pos)
{
    pos = _pos;
}
void Man::setDir(Point _dir)
{
    dir = _dir;
}

float Man::getTurnCost() {
    return turnCost;
}

float Man::getForwardCost() {
    return forwardCost;
}

float Man::getReturnCanCost() {
    return returnCanCost;
}

float Man::getGetCanCost() {
    return getCanCost;
}
