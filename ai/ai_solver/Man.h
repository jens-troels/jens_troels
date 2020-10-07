//
// Created by jeh on 11/27/18.
//

#include "Point.h"

#ifndef AI_SOLVER_MAN_H
#define AI_SOLVER_MAN_H



class Man {
public:
    Man();
    Man(Point _pos, Point _dir, float _turnCost = 1, float _forwardCost = 1, float _returnCanCost = 1, float _getCanCost = 1);
    Point getPos();
    Point getDir();
    void setPos(Point _pos);
    void setDir(Point _dir);
    float getTurnCost();
    float getForwardCost();
    float getReturnCanCost();
    float getGetCanCost();

private:
    Point pos;
    Point dir;
    float turnCost;
    float forwardCost;
    float returnCanCost;
    float getCanCost;

};
#endif //AI_SOLVER_MAN_H
