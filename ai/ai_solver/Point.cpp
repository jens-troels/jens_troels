//
// Created by jeh on 11/26/18.
//

#include "Point.h"


Point::Point(){

}

Point::Point(unsigned int _x, unsigned int _y, char _id)
{
    x = _x;
    y = _y;
    id = _id;
}
unsigned int Point::getX()
{
   return x;
}

unsigned int Point::getY()
{
    return y;
}
char Point::getID()
{
    return id;
}
void Point::setX(unsigned int _x)
{
    x = _x;
}
void Point::setY(unsigned int _y)
{
    y = _y;
}
void Point::setID(char _id)
{
    id = _id;
}


