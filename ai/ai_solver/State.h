//
// Created by jeh on 11/26/18.
//

#ifndef AI_SOLVER_STATE_H
#define AI_SOLVER_STATE_H

#include "Man.h"
#include <cstdlib>
#include <iostream>
#include <vector>


class State {
public:
    State();
    State(vector<Point> _J, Man _M, vector<Point> _G, int _width, int _depth);
    vector<Point> getJewels();
    vector<Point> getGoals();
    Man getMan();


private:
    vector<Point> J;
    vector<Point> G;
    Man M;
    int width;
    int depth;
};


#endif //AI_SOLVER_STATE_H
