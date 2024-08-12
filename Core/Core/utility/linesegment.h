#ifndef CORE_LINESEGMENT_H
#define CORE_LINESEGMENT_H

#include <Core/utility/boundingbox.h>
#include <Core/utility/vector2.h>

#include <vector>

namespace core {

template<typename T>
class TwoDArray;
class IntCoord;

struct LineSegment
{
    Vector2 a;
    Vector2 b;

    LineSegment();
    LineSegment( const Vector2&, const Vector2& );

    BoundingBoxd bounds() const;
    LineSegment reverse() const;
    Vector2 asVec() const;
    double length() const;
    Vector2 midpoint() const;

    /// 't' should be in [0,1].
    Vector2 pos( double t ) const;

    /// Assuming 'p' lies (approximately) on 'this', return the corresponding T value clamped to [0,1].
    double t( const Vector2& p ) const;

    /// Extend the segment in one or both directions.
    void extend( double moveABackBy, double moveBForwardBy );

    static LineSegment fromPosAndDir( const Vector2& pos, const Vector2& dir );
};

std::vector< IntCoord > rasterizeSegment_Bresenham( const Vector2& a, const Vector2& b );
std::vector< IntCoord > rasterizeSegment_Bresenham( const IntCoord& a, const IntCoord& b );
std::vector< IntCoord > rasterizeSegment_midpoint( const Vector2& a, const Vector2& b );
std::vector< IntCoord > rasterizeSegment_midpoint( const IntCoord& a, const IntCoord& b );
std::vector< IntCoord > rasterizeSegment_floatingPoint( const Vector2& a, const Vector2& b );

} // core

#endif // #include guard
