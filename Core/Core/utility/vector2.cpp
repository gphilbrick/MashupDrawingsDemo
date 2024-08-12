#include "vector2.h"
#include "mathutility.h"

#include <cmath>

namespace core {

Vector2::Vector2() : Vector2( 0, 0 )
{
}

Vector2::Vector2( double x, double y )
{
    _x = x;
    _y = y;
}

double Vector2::manhattanDist( const Vector2& a, const Vector2& b )
{
    return std::fabs( a.x() - b.x() ) + std::fabs( a.y() - b.y() );
}

Vector2 Vector2::changeBasis( const Vector2& i, const Vector2& j, const Vector2& input )
{
    return Vector2( i._x * input._x + i._y * input._y, j._x * input._x + j._y * input._y );
}

Vector2 Vector2::undoChangeBasis( const Vector2& i, const Vector2& j, const Vector2& input )
{
    return Vector2( i._x * input._x + j._x * input._y, i._y * input._x + j._y * input._y );
}

double Vector2::crossProductZ( const Vector2& a, const Vector2& b )
{
    return a.x() * b.y() - a.y() * b.x();
}

bool Vector2::operator == (const Vector2& other ) const
{
    return _x == other._x && _y == other._y;
}

bool Vector2::operator != (const Vector2& other ) const
{
    return !( *this == other );
}

Vector2& Vector2::operator += (const Vector2& other)
{
    _x += other._x;
    _y += other._y;
    return *this;
}

Vector2& Vector2::operator -= (const Vector2& other)
{
    _x -= other._x;
    _y -= other._y;
    return *this;
}

Vector2& Vector2::operator *= (double fac)
{
    _x *= fac;
    _y *=fac;
    return *this;
}

Vector2& Vector2::operator /= (double fac)
{
    _x /= fac;
    _y /= fac;
    return *this;
}

const Vector2 Vector2::operator + (const Vector2& b) const
{
    Vector2 a = *this;
    a += b;
    return a;
}

const Vector2 Vector2::operator - (const Vector2& b) const
{
    Vector2 a = *this;
    a -= b;
    return a;
}

const Vector2 Vector2::operator * (const double fac) const
{
    Vector2 a= *this;
    a*=fac;
    return a;
}

const Vector2 Vector2::operator / (const double fac) const
{
    Vector2 a = *this;
    a/=fac;
    return a;
}

void Vector2::scale( const Vector2& scaleBy )
{
    _x *= scaleBy.x();
    _y *= scaleBy.y();
}

Vector2 Vector2::perpendicular( bool cw, bool originAtTopLeft ) const
{
    auto copy = *this;
    copy.turnPerpendicular( cw, originAtTopLeft );
    return copy;
}

void Vector2::turnPerpendicular( bool cw, bool originAtTopLeft )
{
    cw = cw ^ originAtTopLeft;
    if( cw ) {
        const double temp = _x;
        _x = _y;
        _y = -temp;
    } else {
        const double temp = _x;
        _x = -_y;
        _y = temp;
    }
}

void Vector2::normalize()
{
    const double len = length();
    if( mathUtility::closeEnoughToZero( len ) ) return;
    _x /= len;
    _y /= len;
}

double Vector2::length() const
{
    double sum= _x*_x + _y*_y;
    return sqrt(sum);
}

double Vector2::dot(const Vector2& a, const Vector2& b)
{
    return a._x*b._x + a._y*b._y;
}

void Vector2::setX(double x)
{
    _x=x;
}

void Vector2::setY(double y)
{
    _y=y;
}

Vector2 Vector2::normal( double x, double y )
{
    Vector2 ret( x, y );
    ret.normalize();
    return ret;
}

} // core
