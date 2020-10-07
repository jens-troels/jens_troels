//
// Created by jeh on 11/26/18.
//

#include "State.h"
#include <limits>
#include <queue>
#include <map>
#include "hungarian.hpp"


#ifndef AI_SOLVER_SOKOBANSOLVER_H
#define AI_SOLVER_SOKOBANSOLVER_H

#define MAPLOGSIZE 2   //2 means a max size of 100

class SokobanSolver {

    struct mAStar
    {
        string key;
        float cost;
        float heuristicCost;
        vector<Point> jewels;
        string route;
        Man robot;
//        mAStar* cameFrom;


    };

    struct mAStar* newmAstar(mAStar* _cameFrom, vector<Point> _jewels, Man _robot)
    {
        // Allocate memory for new point
        mAStar* mAStar = new SokobanSolver::mAStar;

        // Assign data to this point
        mAStar->cost = std::numeric_limits<float>::infinity();
        mAStar->heuristicCost = std::numeric_limits<float>::infinity();
        mAStar->route = "";
        mAStar->jewels = _jewels;
        mAStar->robot = _robot;

//        // Initialize cameFrom as NULL
//        if(_cameFrom==NULL)
//            mAStar->cameFrom = NULL;
//        else
//            mAStar->cameFrom = _cameFrom;

        return mAStar;
    }

    struct jgAStar
    {
        int x;
        int y;
        float cost;
        float heuristicCost;
        char value;
        jgAStar* cameFrom;
        vector<Point> jewels;
        string route;
        bool closedSet;
        Man robot;
        Point direction;
        bool openSet;
    };

    struct jgAStar* newjgAstar(int x, int y, char value)
    {
        // Allocate memory for new point
        jgAStar* jgAStar = new SokobanSolver::jgAStar;

        // Assign data to this point
        jgAStar->x = x;
        jgAStar->y = y;
        jgAStar->cost = std::numeric_limits<float>::infinity();
        jgAStar->heuristicCost = std::numeric_limits<float>::infinity();
        jgAStar->value = value;
        jgAStar->closedSet = false;
        jgAStar->direction.setX(0);
        jgAStar->direction.setY(0);
        jgAStar->route = "";
        jgAStar->openSet = false;

        // Initialize cameFrom as NULL
        jgAStar->cameFrom = NULL;

        return jgAStar;
    }


    struct aStarPoint
    {
        int x;
        int y;
        float cost;
        float heuristicCost;
        aStarPoint* cameFrom;
        char value;
        Point direction;
        string route;
        vector<Point> jewels;
        bool closedSet;
        bool openSet;
        int gotJewel;
    };

    struct aStarPoint* newaStarPoint(int x, int y, char value)
    {
        // Allocate memory for new point
        aStarPoint* aStarPoint = new SokobanSolver::aStarPoint;

        // Assign data to this point
        aStarPoint->x = x;
        aStarPoint->y = y;
        aStarPoint->cost = std::numeric_limits<float>::infinity();
        aStarPoint->heuristicCost = std::numeric_limits<float>::infinity();
        aStarPoint->value = value;
        aStarPoint->direction.setX(0);
        aStarPoint->direction.setY(0);
        aStarPoint->route = "";
        aStarPoint->closedSet = false;
        aStarPoint->openSet = false;
        aStarPoint->gotJewel = -1;

        // Initialize cameFrom as NULL
        aStarPoint->cameFrom = NULL;

        return(aStarPoint);
    };

public:
    SokobanSolver();
    SokobanSolver(string inputmap);
    SokobanSolver::mAStar* mAStarSolver();
    void printRouteJ(SokobanSolver::jgAStar* cPoint);
    SokobanSolver::aStarPoint* robotAStar(Man robot, Point goal, SokobanSolver::jgAStar* jgNode, int dontMove);
    SokobanSolver::jgAStar* jewelGoalAStar(Man robot, vector<Point> _jewels, Point goal, Point jewel);
    vector<vector<Point>> jPotDeadlocks(vector<Point> J);
    void checkDeadlock();
    vector<Point> jewelGoalCost(Hungarian::Matrix cdMatrix);
    bool checkChar(char input);
    float heuristicDistance(float xStart, float yStart, float xGoal, float yGoal);
    void printMap(vector<vector<char>> _map);
    void initmAStar(SokobanSolver::mAStar* mastarPoint, float previousMoveCost, string _route);
    void testFunc();
    char changeDir(Point cdir, Point ddir);
    int  isJewel(int x, int y, vector<Point> jewels);
    int isJewelDeadlock(int x, int y, vector<Point> jewels);
    void printRoute(SokobanSolver::aStarPoint* cPoint);
    bool isJewelDeadlockS(int x, int y, vector<Point> jewels, int thisJewel);
    string generateKey(SokobanSolver::mAStar* point);


private:
    vector<vector<char>> map;
    int depth;
    int width;
    State startConf;
    ::map<string, mAStar*> stateTableOpen;
    ::map<string, bool> stateTableClosed;

};




#endif //AI_SOLVER_SOKOBANSOLVER_H
