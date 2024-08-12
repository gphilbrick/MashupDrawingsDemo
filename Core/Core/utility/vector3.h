#ifndef CORE_VECTOR3_H
#define CORE_VECTOR3_H

#include <array>

namespace core {

class Vector2;

class Vector3
{
public:
    explicit Vector3( double x = 0, double y = 0, double z = 0 );
    explicit Vector3( const std::array< double, 3 >& );
    Vector3( const Vector2& xy, double z );
    double x() const { return _x; }
    double y() const { return _y; }
    double z() const { return _z; }
    double r() const { return _x; }
    double g() const { return _y; }
    double b() const { return _z; }

    void setX(double x);
    void setY(double y);
    void setZ(double z);

    bool operator == ( const Vector3& ) const;
    bool operator != ( const Vector3& ) const;
    Vector3& operator += (const Vector3&);
    Vector3& operator -= (const Vector3&);
    Vector3& operator *= (double fac);
    Vector3& operator /= (double fac);
    const Vector3 operator + (const Vector3&) const;
    const Vector3 operator - (const Vector3&) const;
    const Vector3 operator * (const double fac) const;
    const Vector3 operator / (const double fac) const;
    const double& operator[](int index) const;

    double length() const;
    void normalize();
    static double dot(const Vector3& a, const Vector3& b);
    static Vector3 cross(const Vector3& a, const Vector3& b);
private:

    double _x;
    double _y;
    double _z;
};

} // core

#endif // #include guard
