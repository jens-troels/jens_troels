//
// Created by jeh on 11/26/18.
//

#include <cmath>
#include "SokobanSolver.h"


SokobanSolver::SokobanSolver()
{

}


void SokobanSolver::checkDeadlock()
{

    Point legalMove;
    vector<Point> deadLock;
    int inf = 0;
    for(int x = 0; x < map.size(); x++)
    {
        for(int y = 0; y < map.at(x).size(); y++)
        {
            if(x-1>=0 && x+1<map.size() && y-1>=0 && y+1<map.at(x).size() && map.at(x).at(y) != 'X'){
                if(map.at(x-1).at(y) == 'X' && map.at(x).at(y-1) == 'X' && map.at(x).at(y) != 'G')
                    map.at(x).at(y) = 'D';
                if(map.at(x-1).at(y) == 'X' && map.at(x).at(y+1) == 'X' && map.at(x).at(y) != 'G')
                    map.at(x).at(y) = 'D';
                if(map.at(x+1).at(y) == 'X' && map.at(x).at(y+1) == 'X' && map.at(x).at(y) != 'G')
                    map.at(x).at(y) = 'D';
                if(map.at(x+1).at(y) == 'X' && map.at(x).at(y-1) == 'X' && map.at(x).at(y) != 'G')
                    map.at(x).at(y) = 'D';
            }
            else{
                if(map.at(x).at(y) != 'X')
                    map.at(x).at(y) = 'D';
            }

        }
    }

//    for(int x = 0; x < map.size(); x++)
//    {
//        for(int y = 0; y < map.at(x).size(); y++)
//        {
//
//            if(x-1>=0 && x+1<map.size() && y-1>=0 && y+1<map.at(x).size() && map.at(x).at(y) != 'X' && map.at(x).at(y) != 'G')
//            {
//                legalMove = Point(x,y,'q');
//                deadLock.push_back(legalMove);
//                inf = 1;
//                for(int m = 0; m < startConf.getGoals().size(); m++)
//                {
//                    SokobanSolver::jgAStar* check = jewelGoalAStar(startConf.getMan(), deadLock, startConf.getGoals().at(m), legalMove);
//                    if(check->cost != std::numeric_limits<float>::infinity())
//                        inf = 0;
//                    delete check;
//                }
//                if(inf == 1)
//                    map.at(x).at(y) = 'D';
//                deadLock.clear();
//            }
//
//        }
//    }


}


SokobanSolver::SokobanSolver(string _map)
{

    vector<char> m;
    vector<Point> J;
    vector<Point> G;
    Man M;
    int line = 0;

    width = ((int)_map[0]-'0')*10 + (int)_map[1]-'0';
    depth = ((int)_map[3]-'0')*10 + (int)_map[4]-'0';
    int jewels = ((int)_map[6]-'0')*10 + (int)_map[7]-'0';

    _map.erase(0,9);
    _map.erase(_map.size());
    for(int i = 0; i < _map.size(); i++)
    {
        if(_map[i] == '\n')
        {
            map.push_back(m);
            m.clear();
            line++;
        }
        else
            if(_map[i] == 'J')
            {
                m.push_back('.');
                Point jx(line,i%(width+1),'J');
                J.push_back(jx);
            }
            else if (_map[i] == 'M')
            {
                m.push_back('.');
                M = Man(Point(line,i%(width+1),'M'),Point(0,-1, 'd'));
            }
            else if (_map[i] == 'G')
            {
                m.push_back('G');
                Point gx(line,i%(width+1),'G');
                G.push_back(gx);
            }

            else
                m.push_back(_map[i]);

    }

    startConf = State(J, M, G, width, depth);
    checkDeadlock();





}

SokobanSolver::mAStar* SokobanSolver::mAStarSolver()
{
    SokobanSolver::mAStar* startPoint = newmAstar(NULL, startConf.getJewels(), startConf.getMan());
    startPoint->key = generateKey(startPoint);
    initmAStar(startPoint,0,"");
    //cout<<startPoint->heuristicCost<<endl;
    stateTableOpen.insert({startPoint->key, startPoint});
    auto compare = [](SokobanSolver::mAStar* lhs, SokobanSolver::mAStar* rhs)
    {
        return lhs->heuristicCost >= rhs->heuristicCost;
    };

    std::priority_queue<SokobanSolver::mAStar*, std::vector<SokobanSolver::mAStar*>, decltype(compare) > openSet(compare);  //Minimum priority queue
    openSet.push(startPoint);

    while(openSet.size()!=0){
        SokobanSolver::mAStar* currentPoint = openSet.top();
        openSet.pop();
        stateTableOpen.erase(currentPoint->key);
        if(currentPoint->cost==currentPoint->heuristicCost){
            std::map<string, mAStar*>::iterator it;
            std::map<string, char>::iterator iti;
            for (it=stateTableOpen.begin(); it!=stateTableOpen.end(); ++it){
                delete it->second;
            }
//            for (iti=stateTableClosed.begin(); iti!=stateTableClosed.end(); ++it){
//                delete iti->second;
//            }
            return currentPoint;
        }
        stateTableClosed.insert({currentPoint->key, true});

        for (int j = 0; j<currentPoint->jewels.size(); j++){
            for (int x = -1; x < 2; x++) {
                for (int y = -1; y < 2; y++) {
                    if (abs(x) == abs(y))
                        continue;
                    Point desiredPoint = Point(currentPoint->jewels.at(j).getX()+x ,currentPoint->jewels.at(j).getY()+y, 'q');
                    SokobanSolver::jgAStar* jgPoint = jewelGoalAStar(currentPoint->robot, currentPoint->jewels, desiredPoint, currentPoint->jewels.at(j));
                    if(jgPoint->cost == std::numeric_limits<float>::infinity())
                    {
                        delete jgPoint;
                        continue;
                    }

                    SokobanSolver::mAStar* nextPoint = newmAstar(currentPoint, jgPoint->jewels, jgPoint->robot);
                    initmAStar(nextPoint, currentPoint->cost+jgPoint->cost, currentPoint->route+jgPoint->route);
                    //cout << "jewel: "<<j<<" to point x: "<<currentPoint->jewels.at(j).getX()+x<<" y: "<<currentPoint->jewels.at(j).getY()+y<<endl;
                    //cout << "key: "<<currentPoint->key<<endl;
                    cout<<"open: "<<stateTableOpen.size()<<" closed:"<<stateTableClosed.size()<<endl;
                    cout<<currentPoint->cost<<"\t"<<currentPoint->heuristicCost<<endl;
                    if(nextPoint->heuristicCost != std::numeric_limits<float>::infinity() && stateTableClosed.count(nextPoint->key)==0 && stateTableOpen.count(nextPoint->key) == 0){
                        stateTableOpen.insert({nextPoint->key, nextPoint});
                        openSet.push(nextPoint);
                    }
                    else{
                        if(stateTableOpen.count(nextPoint->key) > 0){
                            std::map<string, mAStar*>::iterator tempIt;
                            tempIt = stateTableOpen.find(nextPoint->key);
                            if(tempIt->second->heuristicCost > nextPoint->heuristicCost){
                                stateTableOpen.erase(tempIt);
                                stateTableOpen.insert({nextPoint->key, nextPoint});
                                openSet.push(nextPoint);
                            }
                            else
                                delete nextPoint;
                        }
                        else
                            delete nextPoint;
                    }

                    delete jgPoint;
                }

            }
        }
        delete currentPoint;
    }
    std::map<string, mAStar*>::iterator it;
    for (it=stateTableOpen.begin(); it!=stateTableOpen.end(); ++it){
        delete it->second;
    }
//    for (it=stateTableClosed.begin(); it!=stateTableClosed.end(); ++it){
//        delete it->second;
//    }

    return startPoint;

}

void SokobanSolver::initmAStar(SokobanSolver::mAStar* mastarPoint, float previousMoveCost, string _route)
{
    vector<Point> goals = startConf.getGoals();
    vector<Point> jewels = mastarPoint->jewels;
    Hungarian::Matrix hungTable;
    hungTable.resize(jewels.size());
    for(int jewel = 0; jewel < jewels.size(); jewel++)
    {
        for(int goal = 0; goal<goals.size(); goal++)
        {
            //hungTable.at(jewel).push_back(int(heuristicDistance(jewels.at(jewel).getX(),jewels.at(jewel).getY(),goals.at(goal).getX(),goals.at(goal).getY())));
            //cout <<hungTable.at(jewel).at(goal)<<"\t";

            //Robot Man heuristic
            SokobanSolver::jgAStar* tempJG = newjgAstar(0,0,'q');
            SokobanSolver::aStarPoint* tempPoint = robotAStar(Man(jewels.at(jewel),Point(0,1,'p')),goals.at(goal),tempJG, -2);
            hungTable.at(jewel).push_back(int(tempPoint->cost*1));


            //jG heuristic
//            vector<Point> hh;
//            hh.push_back(jewels.at(jewel));
//            SokobanSolver::jgAStar* tempJG = SokobanSolver::jewelGoalAStar(mastarPoint->robot, hh, goals.at(goal), jewels.at(jewel));
//            hungTable.at(jewel).push_back(int(tempJG->cost*1));

            delete tempJG;
            delete tempPoint;
        }
        //cout <<endl;
    }
    float totalCost = Hungarian::Solve(hungTable, Hungarian::MODE_MINIMIZE_COST).totalCost;
    //cout << Hungarian::Solve(hungTable, Hungarian::MODE_MINIMIZE_COST).success<<endl;
    mastarPoint->heuristicCost = totalCost + previousMoveCost;
    mastarPoint->cost = previousMoveCost;
    mastarPoint->route = _route;
    mastarPoint->key = generateKey(mastarPoint);
    hungTable.clear();

}

vector<Point> SokobanSolver::jewelGoalCost(Hungarian::Matrix cdMatrix)
{
    vector<Point> output;
    Hungarian::Matrix result;
    result = Hungarian::Solve(cdMatrix,Hungarian::MODE_MINIMIZE_COST).assignment;
    for (int i = 0; i < result.size(); ++i) {
        for(int j = 0; j < result.at(i).size(); j++)
        {
            if(result.at(i).at(j) == 1)
            {
                output.push_back(Point(i,j,'p'));
            }
        }
    }
}



string SokobanSolver::generateKey(SokobanSolver::mAStar *point) {
    string key;
    //int key = pow(10,0)*(point->robot.getDir().getX()+1)+pow(10,1)*(point->robot.getDir().getY()+1)+pow(10,1*MAPLOGSIZE)*point->robot.getPos().getX()+pow(10,2*MAPLOGSIZE)*point->robot.getPos().getY();
    for (int i=point->jewels.size()-1; i>=0; i--)
    {
        for(int j=MAPLOGSIZE-1; j>=0; j--){
            key += char('0'+(point->jewels.at(i).getX()/int(pow(10,j)))%10);
        }
        for(int j=MAPLOGSIZE-1; j>=0; j--){
            key += char('0'+(point->jewels.at(i).getY()/int(pow(10,j)))%10);
        }
    }
    //key += pow(10,3+(2*i)*MAPLOGSIZE)*point->jewels.at(i).getX()+pow(10,3+1+(2*i)*MAPLOGSIZE)*point->jewels.at(i).getY();
    for(int j=MAPLOGSIZE-1; j>=0; j--){
        key += char('0'+(point->robot.getPos().getX()/int(pow(10,j)))%10);
    }
    for(int j=MAPLOGSIZE-1; j>=0; j--){
        key += char('0'+(point->robot.getPos().getY()/int(pow(10,j)))%10);
    }
    key += char('0'+point->robot.getDir().getX()+1);
    key += char('0'+point->robot.getDir().getY()+1);
    return key;
}

SokobanSolver::jgAStar* SokobanSolver::jewelGoalAStar(Man robot, vector<Point> _jewels, Point goal, Point jewel) {
    int xGoal = goal.getX();
    int yGoal = goal.getY();
    int xJewel = jewel.getX();
    int yJewel = jewel.getY();

    vector<vector<SokobanSolver::jgAStar *>> aStarMap;
    for (int x = 0; x < map.size(); x++) {
        vector<SokobanSolver::jgAStar *> aStarLine;
        aStarMap.push_back(aStarLine);
        for (int y = 0; y < map.at(x).size(); y++) {
            aStarMap.at(x).push_back(newjgAstar(x, y, map.at(x).at(y)));
        }
    }

    //Create compare function for aStarPoint pointers
    auto compare = [](SokobanSolver::jgAStar *lhs, SokobanSolver::jgAStar *rhs) {
        return lhs->heuristicCost >= rhs->heuristicCost;
    };

    std::priority_queue<SokobanSolver::jgAStar *, std::vector<SokobanSolver::jgAStar *>, decltype(compare)> openSet(
            compare);  //Minimum priority queue

    //Set cost, heuristicCost, jewels and direction for the start position of the jewel
    aStarMap.at(xJewel).at(yJewel)->cost = 0;
    aStarMap.at(xJewel).at(yJewel)->heuristicCost = heuristicDistance(xJewel, yJewel, xGoal, yGoal);
    aStarMap.at(xJewel).at(yJewel)->jewels = _jewels;
    aStarMap.at(xJewel).at(yJewel)->robot = robot;

    int jewelID = isJewel(xJewel, yJewel, aStarMap.at(xJewel).at(yJewel)->jewels);

    openSet.push(aStarMap.at(xJewel).at(yJewel));

    while (openSet.size() != 0) {
        //Pop the aStarPoint with the lowest heuristic cost from the priority queue
        SokobanSolver::jgAStar *currentPoint = openSet.top();
        openSet.pop();
        //If currentPoint = goal, return currentPoint
        if (currentPoint->x == xGoal && currentPoint->y == yGoal){
            for (auto elements : currentPoint->jewels)
                aStarMap.at(elements.getX()).at(elements.getY())->value = 'J';
            aStarMap.at(currentPoint->x).at(currentPoint->y)->value = 'M';
//            cout<<"Print Map"<<endl;
//            //Print current aStarMap
//            for(int x=0; x<aStarMap.size(); x++){
//                for(int y=0; y<aStarMap.at(x).size(); y++){
//                    cout<<aStarMap.at(x).at(y)->value;
//                }
//                cout<<endl;
//            }
//            printRouteJ(currentPoint);
//            cout << currentPoint->route << endl;

            //Delete pointers
            for(int x=0; x<aStarMap.size(); x++){
                for(int y=0; y<aStarMap.at(x).size(); y++) {
                    if(currentPoint->x != x || currentPoint->y != y)
                        delete aStarMap.at(x).at(y);
                }
            }
            return currentPoint;
        }

        //Put currentPoint into closedSet
        currentPoint->openSet = false;
        currentPoint->closedSet = true;

        //Check up, down, left, right from current point (no diagonals)
        for (int x = -1; x < 2; x++) {
            for (int y = -1; y < 2; y++) {
                if (abs(x) == abs(y))
                    continue;
                //If the neighboor is in the closed set or the value is 'X', continue
                if (aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->closedSet == true || aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->value == 'X')
                    continue;
                //If one of the points next to current is not a deadlock, wall or potential deadlock
                if (aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->value != 'X' &&
                not isJewelDeadlockS(currentPoint->x+x, currentPoint->y+y, currentPoint->jewels, jewelID) &&
                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->value != 'D' &&
                isJewel(currentPoint->x+x, currentPoint->y+y, currentPoint->jewels) == -1)
                {
                    //Check if robot can stand on the opposite position
                    if(aStarMap.at(currentPoint->x-x).at(currentPoint->y-y)->value != 'X' && isJewel(currentPoint->x-x,currentPoint->y-y,currentPoint->jewels) == -1)
                    {
                        //Update your aStarMap with information from robotAStar and step with jewel
                        SokobanSolver::aStarPoint* tempPoint = robotAStar(currentPoint->robot,Point(currentPoint->x-x,currentPoint->y-y,'g'),currentPoint, jewelID); // change this to jgPoint
                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels = tempPoint->jewels;
                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot = Man(Point(tempPoint->x,tempPoint->y,'M'),tempPoint->direction);
                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route=currentPoint->route;
                        //Check to see if you robot has moved your jewel while going to its position
                        if(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(jewelID).getX() == currentPoint->jewels.at(jewelID).getX() &&
                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(jewelID).getY() == currentPoint->jewels.at(jewelID).getY())
                        {
                            //Depending on direction, update aStarMap
                            char dir = changeDir(tempPoint->direction,Point(x,y,'d'));
                            float tempCost=0;
                            string tempRoute="";
                            switch (dir){
                                case 's':
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cameFrom = currentPoint;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction = Point(x,y,'d');


                                    if(currentPoint->cameFrom != NULL &&
                                            aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction.getX() == currentPoint->direction.getX() &&
                                       aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction.getY() == currentPoint->direction.getY())
                                    {
                                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route.erase(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route.size()-1);
                                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route += "fR";
                                        tempCost -=robot.getReturnCanCost();
                                    }
                                    else{
                                        tempCost += robot.getGetCanCost()+robot.getReturnCanCost();
                                        tempRoute += "GR";
                                    }
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot = Man(Point(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot.getPos().getX()+x,aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot.getPos().getY()+y,'p'),Point(x,y,'d'));
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cost = currentPoint->cost+tempPoint->cost+tempCost;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route += tempPoint->route+tempRoute;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->heuristicCost = heuristicDistance(currentPoint->x+x, currentPoint->y+y, xGoal, yGoal) + aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cost;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(jewelID).setX(currentPoint->x+x);
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(jewelID).setY(currentPoint->y+y);
                                    break;
                                case 'r':
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cameFrom = currentPoint;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction = Point(x,y,'d');


                                    if(currentPoint->cameFrom != NULL &&
                                       aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction.getX() == currentPoint->direction.getX() &&
                                       aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction.getY() == currentPoint->direction.getY())
                                    {
                                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route.erase(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route.size()-1);
                                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route += "fR";
                                        tempCost -=robot.getReturnCanCost();
                                    }
                                    else{
                                        tempCost += robot.getGetCanCost()+robot.getReturnCanCost();
                                        tempRoute += "GR";
                                    }
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot = Man(Point(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot.getPos().getX()+x,aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot.getPos().getY()+y,'p'),Point(x,y,'d'));
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cost = currentPoint->cost+tempPoint->cost+tempCost+robot.getTurnCost();
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route += tempPoint->route+"r"+tempRoute;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->heuristicCost = heuristicDistance(currentPoint->x+x, currentPoint->y+y, xGoal, yGoal) + aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cost;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(jewelID).setX(currentPoint->x+x);
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(jewelID).setY(currentPoint->y+y);
                                    break;
                                case 'l':
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cameFrom = currentPoint;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction = Point(x,y,'d');


                                    if(currentPoint->cameFrom != NULL &&
                                       aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction.getX() == currentPoint->direction.getX() &&
                                       aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction.getY() == currentPoint->direction.getY())
                                    {
                                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route.erase(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route.size()-1);
                                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route += "fR";
                                        tempCost -=robot.getReturnCanCost();
                                    }
                                    else{
                                        tempCost += robot.getGetCanCost()+robot.getReturnCanCost();
                                        tempRoute += "GR";
                                    }
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot = Man(Point(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot.getPos().getX()+x,aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot.getPos().getY()+y,'p'),Point(x,y,'d'));
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cost = currentPoint->cost+tempPoint->cost+tempCost+robot.getTurnCost();
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route += tempPoint->route+"l"+tempRoute;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->heuristicCost = heuristicDistance(currentPoint->x+x, currentPoint->y+y, xGoal, yGoal) + aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cost;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(jewelID).setX(currentPoint->x+x);
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(jewelID).setY(currentPoint->y+y);
                                    break;
                                case 't':
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cameFrom = currentPoint;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction = Point(x,y,'d');


                                    if(currentPoint->cameFrom != NULL &&
                                       aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction.getX() == currentPoint->direction.getX() &&
                                       aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction.getY() == currentPoint->direction.getY())
                                    {
                                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route.erase(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route.size()-1);
                                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route += "fR";
                                        tempCost -=robot.getReturnCanCost();
                                    }
                                    else{
                                        tempCost += robot.getGetCanCost()+robot.getReturnCanCost();
                                        tempRoute += "GR";
                                    }
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot = Man(Point(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot.getPos().getX()+x,aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->robot.getPos().getY()+y,'p'),Point(x,y,'d'));
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cost = currentPoint->cost+tempPoint->cost+tempCost+robot.getTurnCost()*2;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route += tempPoint->route+"t"+tempRoute;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->heuristicCost = heuristicDistance(currentPoint->x+x, currentPoint->y+y, xGoal, yGoal) + aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cost;
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(jewelID).setX(currentPoint->x+x);
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(jewelID).setY(currentPoint->y+y);
                                    break;
                            }
                        }
                        delete tempPoint;
                    }
                }
                if(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->openSet == false && aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->heuristicCost != std::numeric_limits<float>::infinity())
                {
                    openSet.push(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y));
                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->openSet = true;
                }


            }

        }

    }

    SokobanSolver::jgAStar* failed = newjgAstar(0,0,'f');
    failed->cost = std::numeric_limits<float>::infinity();
    failed->heuristicCost = std::numeric_limits<float>::infinity();

    for(int x=0; x<aStarMap.size(); x++){
        for(int y=0; y<aStarMap.at(x).size(); y++) {
            delete aStarMap.at(x).at(y);
        }
    }

    return failed;
}

SokobanSolver::aStarPoint* SokobanSolver::robotAStar(Man robot, Point goal, SokobanSolver::jgAStar* jgNode, int dontMove) {
    int xGoal=goal.getX();
    int yGoal=goal.getY();
    //Create AStarMap
//    cout << "JEPPE"<<endl;
    vector<vector<SokobanSolver::aStarPoint*>> aStarMap;
    for (int x=0; x<map.size(); x++){
        vector<SokobanSolver::aStarPoint*> aStarLine;
        aStarMap.push_back(aStarLine);
        for (int y=0; y<map.at(x).size(); y++){
            aStarMap.at(x).push_back(newaStarPoint(x, y, map.at(x).at(y)));
        }
    }



    //Create compare function for aStarPoint pointers
    auto compare = [](SokobanSolver::aStarPoint* lhs, SokobanSolver::aStarPoint* rhs)
    {
        return lhs->heuristicCost >= rhs->heuristicCost;
    };

    std::priority_queue<SokobanSolver::aStarPoint*, std::vector<SokobanSolver::aStarPoint*>, decltype(compare) > openSet(compare);  //Minimum priority queue

    //Set cost, heuristicCost, jewels and direction for the start position of the robot
    aStarMap.at(robot.getPos().getX()).at(robot.getPos().getY())->cost = 0;
    aStarMap.at(robot.getPos().getX()).at(robot.getPos().getY())->heuristicCost = heuristicDistance(robot.getPos().getX(), robot.getPos().getY(),xGoal, yGoal);
    aStarMap.at(robot.getPos().getX()).at(robot.getPos().getY())->direction = robot.getDir();
    aStarMap.at(robot.getPos().getX()).at(robot.getPos().getY())->jewels = jgNode->jewels;


    openSet.push(aStarMap.at(robot.getPos().getX()).at(robot.getPos().getY()));


    while (openSet.size()!=0){

        //Pop the aStarPoint with the lowest heuristic cost from the priority queue
        SokobanSolver::aStarPoint* currentPoint = openSet.top();
        openSet.pop();
        //If currentPoint = goal, return currentPoint
        if (currentPoint->x == xGoal && currentPoint->y == yGoal){
            if(currentPoint->gotJewel!=-1){
                //int x = currentPoint->direction.getX();
                //int y = currentPoint->direction.getY();
                if(aStarMap.at(currentPoint->x+currentPoint->direction.getX()).at(currentPoint->y+currentPoint->direction.getY())->value == '.' &&
                   isJewel(currentPoint->x+currentPoint->direction.getX(),currentPoint->y+currentPoint->direction.getY(),currentPoint->jewels)==-1 &&
                        isJewelDeadlockS(currentPoint->x+currentPoint->direction.getX(),currentPoint->y+currentPoint->direction.getY(), currentPoint->jewels, currentPoint->gotJewel)){
                    currentPoint->cost += robot.getReturnCanCost();

                    currentPoint->jewels.at(currentPoint->gotJewel).setX(currentPoint->x+currentPoint->direction.getX());
                    currentPoint->jewels.at(currentPoint->gotJewel).setY(currentPoint->y+currentPoint->direction.getY());
                    currentPoint->gotJewel = -1;
                }
                else{
                    currentPoint->cost = std::numeric_limits<float>::infinity();
                }
                currentPoint->route += "R";
            }
                for (auto elements : currentPoint->jewels)
                    aStarMap.at(elements.getX()).at(elements.getY())->value = 'J';
            aStarMap.at(currentPoint->x).at(currentPoint->y)->value = 'M';
//           cout<<"Print Map"<<endl;
            //Print current aStarMap
//            for(int x=0; x<aStarMap.size(); x++){
//                for(int y=0; y<aStarMap.at(x).size(); y++){
//                   cout<<aStarMap.at(x).at(y)->value;
//                }
//                cout<<endl;
//            }
//            printRoute(currentPoint);
//            cout << currentPoint->route << endl;

            //Delete pointers
            for(int x=0; x<aStarMap.size(); x++){
                for(int y=0; y<aStarMap.at(x).size(); y++) {
                    if(currentPoint->x != x || currentPoint->y != y)
                        delete aStarMap.at(x).at(y);
                }
            }

            return currentPoint;

        }

        //Put currentPoint into closedSet
        currentPoint->closedSet = true;
        currentPoint->openSet = false;

        for(int x = -1; x<2; x++){
            for(int y = -1; y<2; y++){
                if(abs(x) == abs(y))
                    continue;

                int isAJewel = isJewel(currentPoint->x+x, currentPoint->y+y, currentPoint->jewels); //returns the id of the jewel that is on the position. -1 if there isn't any jewel
                //If the neighboor is in the closed set or the value is 'X' or the jewel we are going to move, continue
                if (aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->closedSet == true || aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->value == 'X' || isAJewel == dontMove)
                    continue;
                //Initilize neighbour and put into openSet
                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels = currentPoint->jewels;
                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->gotJewel = currentPoint->gotJewel;
                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route = currentPoint->route;
                char dir = changeDir(currentPoint->direction,Point(x,y,'d'));
                float tempCost = 0;
                string tempRoute = "";
                //Check if current point has a jewel, gotJewel contains -1 if not, and the id of the jewel if it has one
                if(currentPoint->gotJewel==-1){
                    //Define cost for route
                    switch (dir){
                        case 's':
                            tempCost = robot.getForwardCost();
                            break;
                        case 'r':
                            tempCost = robot.getTurnCost();
                            tempRoute += "r";
                            break;
                        case 'l':
                            tempCost = robot.getTurnCost();
                            tempRoute += "l";
                            break;
                        case 't':
                            tempCost = robot.getTurnCost()*2;
                            tempRoute += "t";
                            break;
                    }

                    //Check for jewels and deadlocks after jewels

                    if(isAJewel>=0){
                        tempCost += robot.getGetCanCost();
                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->gotJewel = isAJewel;
                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(isAJewel).setX(currentPoint->x+x);
                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(isAJewel).setY(currentPoint->y+y);
                        if(aStarMap.at(currentPoint->x+2*x).at(currentPoint->y+2*y)->value == 'X' ||
                                aStarMap.at(currentPoint->x+2*x).at(currentPoint->y+2*y)->value == 'D' ||
                                isJewel(currentPoint->x+2*x,currentPoint->y+2*y,currentPoint->jewels)>=0 ||
                                (isJewelDeadlockS(currentPoint->x+2*x,currentPoint->y+2*y,currentPoint->jewels, isAJewel) )){
                            tempCost = std::numeric_limits<float>::infinity();
//                            cout<<"infinity because x: "<<currentPoint->x<<" y: "<<currentPoint->y<<endl;
                        }
                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route += tempRoute + "G";
                    }
                    else{
                        aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route += tempRoute +"f";
                    }


                }
                else{   //The robot got a jewel
                    int isAJewel = isJewel(currentPoint->x+x, currentPoint->y+y, currentPoint->jewels); //returns the id of the jewel that is on the position. -1 if there isn't any jewel
                    switch (dir){
                        case 's':
                            tempCost = robot.getForwardCost();
                            aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(currentPoint->gotJewel).setX(currentPoint->x+currentPoint->direction.getX());
                            aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(currentPoint->gotJewel).setY(currentPoint->y+currentPoint->direction.getY());
                            if(aStarMap.at(currentPoint->x+2*x).at(currentPoint->y+2*y)->value == 'X' ||
                               aStarMap.at(currentPoint->x+2*x).at(currentPoint->y+2*y)->value == 'D' ||
                               isJewel(currentPoint->x+2*x,currentPoint->y+2*y,currentPoint->jewels)>=0 ||
                               (isJewelDeadlockS(currentPoint->x+2*x,currentPoint->y+2*y,currentPoint->jewels, isAJewel) )){
                                tempCost=std::numeric_limits<float>::infinity();
                            }
                            tempRoute = "f";
                            break;
                        case 'r':
                            if(aStarMap.at(currentPoint->x+currentPoint->direction.getX()).at(currentPoint->y+currentPoint->direction.getY())->value == '.' &&
                                    isJewel(currentPoint->x+currentPoint->direction.getX(),currentPoint->y+currentPoint->direction.getY(),currentPoint->jewels)==-1 &&
                                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->value == '.' &&
                                    isJewel(currentPoint->x+x,currentPoint->y+y,currentPoint->jewels)==-1){
                                tempCost = robot.getTurnCost()+robot.getReturnCanCost();
                                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->gotJewel = -1;
                                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(currentPoint->gotJewel).setX(currentPoint->x+currentPoint->direction.getX());
                                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(currentPoint->gotJewel).setY(currentPoint->y+currentPoint->direction.getY());
                            }
                            else{
                                tempCost = std::numeric_limits<float>::infinity();
                            }
                            tempRoute = "Rrf";
                            break;
                        case 'l':
                            if(aStarMap.at(currentPoint->x+currentPoint->direction.getX()).at(currentPoint->y+currentPoint->direction.getY())->value == '.' &&
                               isJewel(currentPoint->x+currentPoint->direction.getX(),currentPoint->y+currentPoint->direction.getY(),currentPoint->jewels)==-1 &&
                               aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->value == '.' &&
                               isJewel(currentPoint->x+x,currentPoint->y+y,currentPoint->jewels)==-1){
                                tempCost = robot.getTurnCost()+robot.getReturnCanCost();
                                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->gotJewel = -1;
                                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(currentPoint->gotJewel).setX(currentPoint->x+currentPoint->direction.getX());
                                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(currentPoint->gotJewel).setY(currentPoint->y+currentPoint->direction.getY());
                            }
                            else{
                                tempCost = std::numeric_limits<float>::infinity();
                            }
                            tempRoute = "Rlf";
                            break;
                        case 't':
                            if(aStarMap.at(currentPoint->x+currentPoint->direction.getX()).at(currentPoint->y+currentPoint->direction.getY())->value == '.' &&
                               isJewel(currentPoint->x+currentPoint->direction.getX(),currentPoint->y+currentPoint->direction.getY(),currentPoint->jewels)==-1){
                                tempCost = robot.getTurnCost()+2*robot.getReturnCanCost();
                                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->gotJewel = -1;
                                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(currentPoint->gotJewel).setX(currentPoint->x+currentPoint->direction.getX());
                                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->jewels.at(currentPoint->gotJewel).setY(currentPoint->y+currentPoint->direction.getY());
                            }
                            else{
                                tempCost = std::numeric_limits<float>::infinity();
                            }
                            tempRoute = "Rtf";
                            break;
                    }
                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->route += tempRoute;
                }
                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cost = tempCost + currentPoint->cost;
                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->heuristicCost = heuristicDistance(currentPoint->x, currentPoint->y, xGoal, yGoal) + aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cost;
                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->direction = Point(x,y,'d');
                aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->cameFrom = currentPoint;
//                cout<<"checks x: "<<currentPoint->x+x<<" y: "<<currentPoint->y+y<<" value: "<<aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->heuristicCost<< "openSet?=: "<<aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->openSet<<endl;

                if(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->openSet == false && aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->heuristicCost!=std::numeric_limits<float>::infinity()){
                    openSet.push(aStarMap.at(currentPoint->x+x).at(currentPoint->y+y));
                    aStarMap.at(currentPoint->x+x).at(currentPoint->y+y)->openSet = true;

                }




            }
        }
//        cout <<"x: "<<currentPoint->x<<" y: "<<currentPoint->y<<"cost= "<<currentPoint->heuristicCost<<endl;
//        for (auto elements : currentPoint->jewels)
//            aStarMap.at(elements.getX()).at(elements.getY())->value = 'J';
//        aStarMap.at(currentPoint->x).at(currentPoint->y)->value = 'M';
//        cout<<"Print Map"<<endl;
        //Print current aStarMap
//        if(currentPoint->cameFrom!=NULL)
//            cout <<"came from x: "<<currentPoint->cameFrom->x<<" y: "<<currentPoint->cameFrom->y<<endl;
//        for(int x=0; x<aStarMap.size(); x++) {
//            for (int y = 0; y < aStarMap.at(x).size(); y++) {
//                if(x==currentPoint->x && y==currentPoint->y){
//                    cout <<"M";
//                }
//                else{
//                    if(isJewel(x,y,currentPoint->jewels)>=0){
//                        cout<<"J";
//                    }
//                    else
//                        cout << aStarMap.at(x).at(y)->value;
//
//                }
//
//            }
//            cout << endl;
//        }

    }
    SokobanSolver::aStarPoint* failed = newaStarPoint(robot.getPos().getX(),robot.getPos().getY(),'0');
    failed->cost = std::numeric_limits<float>::infinity();
    failed->heuristicCost = std::numeric_limits<float>::infinity();
    failed->jewels = jgNode->jewels;

    for(int x=0; x<aStarMap.size(); x++){
        for(int y=0; y<aStarMap.at(x).size(); y++) {
            delete aStarMap.at(x).at(y);
        }
    }

    return failed;
}

void SokobanSolver::printMap(vector<vector<char>> _map)
{
        for(int i = 0; i < _map.size(); i++){
        for(int j = 0; j < _map.at(i).size(); j++)
        {
            cout << _map.at(i).at(j);
            if(j == width-1)
                cout << '\n';
        }
    }
}

float SokobanSolver::heuristicDistance(float xStart, float yStart, float xGoal, float yGoal) {
    float ans = sqrt(pow(xGoal - xStart, 2) + pow(yGoal - yStart, 2));
    return ans;
}


void SokobanSolver::testFunc()
{

//    Point legalMove(6,2,'q');
//    vector<Point> deadLock;
//    deadLock.push_back(legalMove);
//    for(int m = 0; m < 4; m++)
//    {
//        SokobanSolver::jgAStar* check = jewelGoalAStar(startConf.getMan(), deadLock, startConf.getGoals().at(m), legalMove);
//        cout << check->cost << endl;
//    }

//    jgAStar* check = newjgAstar(10,2,'.');
//    check->jewels = deadLock;
//
//    Man man(Point(8,5,'M'),Point(0,-1,'d'));
//    aStarPoint* temp = SokobanSolver::robotAStar(man,Point(5,1,'G'),check,0);
//    cout << temp->cost << endl;

//    printMap(map);


//    Hungarian::Matrix test;
//    test.resize(3);
//    test.at(0).push_back(1);
//    test.at(0).push_back(10);
//    test.at(0).push_back(1);
//    test.at(1).push_back(25);
//    test.at(1).push_back(2);
//    test.at(1).push_back(35);
//    test.at(2).push_back(3);
//    test.at(2).push_back(20);
//    test.at(2).push_back(3);


//   Hungarian::PrintMatrix(Hungarian::Solve(test, Hungarian::MODE_MINIMIZE_COST).assignment);
   //cout << Hungarian::Solve(test, Hungarian::MODE_MINIMIZE_COST).totalCost << endl;


//    State tmpState = startConf;
//    vector<vector<Point>> tester = jPotDeadlocks(tmpState.getJewels());

    SokobanSolver::mAStar* win = mAStarSolver();
    cout << win->route<<endl;
    delete win;




//    for(int i = 0; i < tester.size(); i++){
//        for(auto d : tester.at(i)){
//            cout << "jewel: " << i << " x: " << d.getX() << " y: " << d.getY() << endl;
//        }
//    }

}

char SokobanSolver::changeDir(Point cdir, Point ddir)
{
    if(cdir.getX() == ddir.getX() && cdir.getY() == ddir.getY())
        return 's';

    if(cdir.getX() == 0 && cdir.getY() == 1)
    {
        if(ddir.getX() == 1 && ddir.getY() == 0)
            return 'r';
        if(ddir.getX() == -1 && ddir.getY() == 0)
            return 'l';
        if(ddir.getX() == 0 && ddir.getY() == -1)
            return 't';
    }

    if(cdir.getX() == 1 && cdir.getY() == 0)
    {
        if(ddir.getX() == 0 && ddir.getY() == -1)
            return 'r';
        if(ddir.getX() == 0 && ddir.getY() == 1)
            return 'l';
        if(ddir.getX() == -1 && ddir.getY() == 0)
            return 't';
    }

    if(cdir.getX() == 0 && cdir.getY() == -1)
    {
        if(ddir.getX() == -1 && ddir.getY() == 0)
            return 'r';
        if(ddir.getX() == 1 && ddir.getY() == 0)
            return 'l';
        if(ddir.getX() == 0 && ddir.getY() == 1)
            return 't';
    }

    if(cdir.getX() == -1 && cdir.getY() == 0)
    {
        if(ddir.getX() == 0 && ddir.getY() == 1)
            return 'r';
        if(ddir.getX() == 0 && ddir.getY() == -1)
            return 'l';
        if(ddir.getX() == 1 && ddir.getY() == 0)
            return 't';
    }

}

bool SokobanSolver::checkChar(char input)
{
    if((int)input >= 48 && (int)input <= 57)
        return true;
    return false;

}

int SokobanSolver::isJewel(int x, int y, vector<Point> jewels) {
    for (int i=0; i<jewels.size(); i++){
        if(jewels.at(i).getX() == x && jewels.at(i).getY()==y)
            return i;
    }
    return -1;
}

vector<vector<Point>> SokobanSolver::jPotDeadlocks(vector<Point> J) {
    vector<vector<Point>> output;
    output.resize(J.size());
    int move = 0;
    for (int i = 0; i < J.size(); i++)
    {
        int x = J.at(i).getX();
        int y = J.at(i).getY();

        if (map.at(x - 1).at(y) == 'X' && map.at(x + 1).at(y) == 'X')
        {
            move = 0;
            while (map.at(x - 1).at(y + move) == 'X' && map.at(x + 1).at(y + move) == 'X' &&
            map.at(x).at(y + move) != 'X') {
                move++;
                output.at(i).push_back(Point(x, y + move, (char) i + '0'));
            }
            move = 0;
            while (map.at(x - 1).at(y - move) == 'X' && map.at(x + 1).at(y - move) == 'X' &&
            map.at(x).at(y - move) != 'X') {
                move++;
                output.at(i).push_back(Point(x, y - move, (char) i + '0'));
            }
        }
        if (map.at(x).at(y - 1) == 'X' && map.at(x).at(y + 1) == 'X')
        {
            move = 0;
            while (map.at(x + move).at(y - 1) == 'X' && map.at(x + move).at(y + 1) == 'X' &&
            map.at(x + move).at(y) != 'X') {
                move++;
                output.at(i).push_back(Point(x + move, y, (char) i + '0'));
            }
            move = 0;
            while (map.at(x - move).at(y - 1) == 'X' && map.at(x - move).at(y + 1) == 'X' &&
            map.at(x - move).at(y) != 'X') {
                move++;
                output.at(i).push_back(Point(x - move, y, (char) i + '0'));
            }
        }
        if (map.at(x - 1).at(y) == 'X') {
            if (map.at(x - 1).at(y - 1) == 'X' && map.at(x).at(y - 1) == '.')
                output.at(i).push_back(Point(x, y - 1, (char) i + '0'));
            if (map.at(x - 1).at(y + 1) == 'X' && map.at(x).at(y + 1) == '.')
                output.at(i).push_back(Point(x, y + 1, (char) i + '0'));
        }
        if (map.at(x).at(y + 1) == 'X') {
            if (map.at(x - 1).at(y + 1) == 'X' && map.at(x - 1).at(y) == '.')
                output.at(i).push_back(Point(x - 1, y, (char) i + '0'));
            if (map.at(x + 1).at(y + 1) == 'X' && map.at(x + 1).at(y) == '.')
                output.at(i).push_back(Point(x + 1, y, (char) i + '0'));
        }
        if (map.at(x + 1).at(y) == 'X') {
            if (map.at(x + 1).at(y + 1) == 'X' && map.at(x).at(y + 1) == '.')
                output.at(i).push_back(Point(x, y + 1, (char) i + '0'));
            if (map.at(x + 1).at(y - 1) == 'X' && map.at(x).at(y - 1) == '.')
                output.at(i).push_back(Point(x, y - 1, (char) i + '0'));
        }
        if (map.at(x).at(y - 1) == 'X') {
            if (map.at(x + 1).at(y - 1) == 'X' && map.at(x + 1).at(y) == '.')
                output.at(i).push_back(Point(x + 1, y, (char) i + '0'));
            if (map.at(x - 1).at(y - 1) == 'X' && map.at(x - 1).at(y) == '.')
                output.at(i).push_back(Point(x - 1, y, (char) i + '0'));
        }
    }
    return output;
}

int SokobanSolver::isJewelDeadlock(int x, int y, vector<Point> jewels) {
    vector<vector<Point>> potDeadlocks =jPotDeadlocks(jewels);
    for (int i = 0; i<potDeadlocks.size(); i++){
        for(int j = 0; j<potDeadlocks.at(i).size(); j++){
            if(potDeadlocks.at(i).at(j).getX() == x && potDeadlocks.at(i).at(j).getY() == y)
                return i;
        }
    }
    return -1;
}

void SokobanSolver::printRoute(SokobanSolver::aStarPoint* cPoint) {
    if(cPoint->cameFrom != NULL)
        printRoute(cPoint->cameFrom);
    cout <<"x: "<<cPoint->x<<" y: "<<cPoint->y<<" got can="<<cPoint->gotJewel<< " Direction x: "<<cPoint->direction.getX()<<" y: "<<cPoint->direction.getY()<<endl;

}

void SokobanSolver::printRouteJ(SokobanSolver::jgAStar* cPoint) {
    if(cPoint->cameFrom != NULL)
        printRouteJ(cPoint->cameFrom);
    cout <<"x: "<<cPoint->x<<" y: "<<cPoint->y<< " Direction x: "<<cPoint->direction.getX()<<" y: "<<cPoint->direction.getY()<<endl;

}

bool SokobanSolver:: isJewelDeadlockS(int x, int y, vector<Point> jewels, int thisJewel) {
    vector<vector<Point>> potDeadlocks =jPotDeadlocks(jewels);
    for (int i = 0; i<potDeadlocks.size(); i++){
        for(int j = 0; j<potDeadlocks.at(i).size(); j++){
            if(i!=thisJewel){
                if(potDeadlocks.at(i).at(j).getX() == x && potDeadlocks.at(i).at(j).getY() == y)
                    return true;
            }

        }
    }
    return false;
}



