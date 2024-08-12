#include "vector3.h"
#include "mathutility.h"

#include <cmath>

namespace core {

Vector3::Vector3( const std::array< double, 3 >& xyz )
{
    _x = xyz[ 0 ];
    _y = xyz[ 1 ];
    _z = xyz[ 2 ];
}

Vector3::Vector3(double x, double y, double z)
{
    _x = x;
    _y = y;
    _z = z;
}

Vector3::Vector3( const Vector2& xy, double z )
{
    _x = xy.x();
    _y = xy.y();
    _z = z;
}

void Vector3::setX(double x)
{
    _x=x;
}

void Vector3::setY(double y)
{
    _y=y;
}

void Vector3::setZ(double z)
{
    _z=z;
}

const double& Vector3::operator[](int index) const
{
    if(index==0) return _x;
    if(index==1) return _y;
    if(index==2) return _z;
    throw std::runtime_error( "Vector3: unrecognized index" );
}

Vector3& Vector3::operator += (const Vector3& other)
{
    _x += other._x;
    _y += other._y;
    _z += other._z;
    return *this;
}

Vector3& Vector3::operator -= (const Vector3& other)
{
    _x -= other._x;
    _y -= other._y;
    _z -= other._z;
    return *this;
}

Vector3& Vector3::operator *= (double fac)
{
    _x *= fac;
    _y *=fac;
    _z *=fac;
    return *this;
}

Vector3& Vector3::operator /= (double fac)
{
    _x /= fac;
    _y /= fac;
    _z /= fac;
    return *this;
}

const Vector3 Vector3::operator + (const Vector3& b) const
{
    Vector3 a = *this;
    a += b;
    return a;
}

const Vector3 Vector3::operator - (const Vector3& b) const
{
    Vector3 a = *this;
    a -= b;
    return a;
}

const Vector3 Vector3::operator * (const double fac) const
{
    Vector3 a= *this;
    a*=fac;
    return a;
}

const Vector3 Vector3::operator / (const double fac) const
{
    Vector3 a = *this;
    a/=fac;
    return a;
}

Vector3 Vector3::cross(const Vector3& a, const Vector3& b)
{
    return Vector3(
                a.y()*b.z() - a.z()*b.y(),
                a.z()*b.x() - a.x()*b.z(),
                a.x()*b.y() - a.y()*b.x()
                );
}


void Vector3::normalize()
{
    double len = length();
    if(mathUtility::closeEnoughToZero(len)) return;
    _x /= len;
    _y /= len;
    _z /= len;
}

double Vector3::length() const
{
    double sum= _x*_x + _y*_y + _z*_z;
    return sqrt(sum);
}

double Vector3::dot(const Vector3& a, const Vector3& b)
{
    return a._x*b._x + a._y*b._y + a._z*b._z;
}

bool Vector3::operator == (const Vector3& other ) const
{
    return _x == other._x && _y == other._y && _z == other._z;
}

bool Vector3::operator != (const Vector3& other ) const
{
    return !( *this == other );
}

} // core
