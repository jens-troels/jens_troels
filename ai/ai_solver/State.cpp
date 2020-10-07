//
// Created by jeh on 11/26/18.
//

#include "State.h"


State::State()
{

}

State::State(vector<Point> _J, Man _M, vector<Point> _G, int _width, int _depth)
{
    J = _J;
    M = _M;
    G = _G;
    depth = _depth;
    width = _width;
}

vector<Point> State::getJewels()
{
    return J;
}

Man State::getMan()
{
    return M;
}


vector <Point> State::getGoals() {
    return G;
}

