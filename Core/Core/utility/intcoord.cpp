#include "intcoord.h"
#include <cmath>

namespace core {

IntCoord::IntCoord()
{
    _x=0;
    _y=0;
}

void IntCoord::setX(int x)
{
    _x=x;
}

void IntCoord::setY(int y)
{
    _y=y;
}


IntCoord IntCoord::fromVector2(const Vector2& v)
{
    return IntCoord((int)v.x(),(int)v.y());
}


IntCoord::IntCoord(int inputX, int inputY)
{
    _x=inputX;
    _y=inputY;
}

IntCoord& IntCoord::operator += (const IntCoord& other)
{
    _x += other._x;
    _y += other._y;
    return *this;
}

IntCoord& IntCoord::operator -= (const IntCoord& other)
{
    _x -= other._x;
    _y -= other._y;
    return *this;
}

Vector2 IntCoord::toVector2() const
{
    return Vector2((double)_x,(double)_y);
}


const IntCoord IntCoord::operator +(const IntCoord& other) const
{
    IntCoord a = *this;
    a += other;
    return a;
}

const IntCoord IntCoord::operator -(const IntCoord& other) const
{
    IntCoord a = *this;
    a -= other;
    return a;
}

bool IntCoord::operator == (const IntCoord& other) const
{
    return _x == other._x && _y == other._y;
}

bool IntCoord::operator != (const IntCoord& other) const
{
    return !( *this == other);
}

bool IntCoord::operator <(const IntCoord& other) const
{
    if(_x<other._x) return true;
    if(_x>other._x) return false;
    if(_y<other._y) return true;
    return false;
}

double IntCoord::cartesianDistance(const IntCoord& a, const IntCoord& b)
{
    double xOff = (double)abs(a.x()-b.x());
    double yOff = (double)abs(a.y()-b.y());
    return sqrt(xOff*xOff + yOff*yOff);
}

IntCoord IntCoord::left() const
{
    return IntCoord{ _x - 1, _y };
}

IntCoord IntCoord::right() const
{
    return IntCoord{ _x + 1, _y };
}

IntCoord IntCoord::below() const
{
    return IntCoord{ _x, _y + 1 };
}

IntCoord IntCoord::above() const
{
    return IntCoord{ _x, _y - 1 };
}

} // core
