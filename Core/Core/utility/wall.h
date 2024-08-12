#ifndef CORE_WALL_H
#define CORE_WALL_H

#include <Core/utility/vector2.h>

namespace core {

struct LineSegment;

/// A line the divides 2D space into in-front and behind.
class Wall
{
public:
    Wall( const Vector2& pointOnWall, const Vector2& wallNormal );
    LineSegment alongWall() const;
    const Vector2& normal() const;
    const Vector2& pointOnWall() const;
    bool inFrontOfWall( const Vector2& ) const;
private:
    Vector2 _onWall;
    Vector2 _norm;
};

} // core

#endif // #include
