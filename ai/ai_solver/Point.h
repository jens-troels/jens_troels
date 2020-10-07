//
// Created by jeh on 11/26/18.
//

#ifndef AI_SOLVER_POINT_H
#define AI_SOLVER_POINT_H

using namespace std;

class Point {
public:
    Point();
    Point(unsigned int _x, unsigned int _y, char _id);
    unsigned int getX();
    unsigned int getY();
    char getID();
    void setX(unsigned int _x);
    void setY(unsigned int _y);
    void setID(char _id);

private:
    unsigned int x;
    unsigned int y;
    char id;


};


#endif //AI_SOLVER_POINT_H
