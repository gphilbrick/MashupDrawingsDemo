#include <utility/wall.h>

#include <utility/linesegment.h>

namespace core {

Wall::Wall( const Vector2& pointOnWall, const Vector2& wallNormal )
{
    _onWall = pointOnWall;
    _norm = wallNormal;
    _norm.normalize();
}

LineSegment Wall::alongWall() const
{
    auto perp = _norm;
    perp.turnPerpendicular();
    return LineSegment{ _onWall, _onWall + perp };
}

const Vector2& Wall::normal() const
{
    return _norm;
}

bool Wall::inFrontOfWall( const Vector2& p ) const
{
    auto toP = p - _onWall;
    toP.normalize();
    return Vector2::dot( toP, _norm ) > 0;
}

const Vector2& Wall::pointOnWall() const
{
    return _onWall;
}

} // core
