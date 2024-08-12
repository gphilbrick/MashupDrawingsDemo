#ifndef CORE_INTCOORD_H
#define CORE_INTCOORD_H

#include "vector2.h"

namespace core {

class IntCoord
{
public:
    IntCoord(int,int);
    IntCoord();
    int x() const { return _x; }
    int y() const { return _y; }

    IntCoord& operator += (const IntCoord&);
    IntCoord& operator -= (const IntCoord&);
    const IntCoord operator + (const IntCoord&) const;
    const IntCoord operator - (const IntCoord&) const;
    bool operator == (const IntCoord&) const;
    bool operator != (const IntCoord&) const;
    bool operator <(const IntCoord&) const;

    IntCoord left() const;
    IntCoord right() const;
    IntCoord below() const;
    IntCoord above() const;

    void setX(int x);
    void setY(int y);

    Vector2 toVector2() const;

    static IntCoord fromVector2(const Vector2&);

    static double cartesianDistance(const IntCoord&, const IntCoord&);

private:
    int _x;
    int _y;
};

} // core

#endif // #include guard
