#ifndef CORE_VECTOR2_H
#define CORE_VECTOR2_H

namespace core {

class Vector2
{
public:
    Vector2();
    Vector2( double, double );
    double x() const { return _x; }
    double y() const { return _y; }

    Vector2& operator += (const Vector2&);
    Vector2& operator -= (const Vector2&);
    Vector2& operator *= (double fac);
    Vector2& operator /= (double fac);
    const Vector2 operator + (const Vector2&) const;
    const Vector2 operator - (const Vector2&) const;
    const Vector2 operator * (const double fac) const;
    const Vector2 operator / (const double fac) const;
    bool operator == ( const Vector2& ) const;
    // For some reason it took me like six years to add this one.
    bool operator != ( const Vector2& ) const;
    double length() const;
    void normalize();
    void turnPerpendicular( bool cw = false, bool originAtTopLeft = false );
    Vector2 perpendicular( bool cw = false, bool originAtTopLeft = false ) const;

    /// Call the X and Y components by the corresponding coordinates in 'scaleBy'.
    void scale( const Vector2& scaleBy );

    static double dot(const Vector2& a, const Vector2& b);
    /// Return 'input' transformed such that if it equals 'i', it becomes (1,0) and if it equals 'j' it
    /// becomes (0,1). 'i' and 'j' must be orthogonal.
    static Vector2 changeBasis( const Vector2& i, const Vector2& j, const Vector2& input );
    /// Reverse the effect of 'changeBasis'. 'i' and 'j' must be orthogonal.
    static Vector2 undoChangeBasis( const Vector2& i, const Vector2& j, const Vector2& input );
    /// 'x' and 'y' cannot both be 0.
    static Vector2 normal( double x, double y );

    static Vector2 lerp( const Vector2& a, const Vector2& b, double f )
    {
        const auto diff = b - a;
        return a + diff *  f;
    }

    static double manhattanDist( const Vector2& a, const Vector2& b );

    /// Return the Z component of the cross product of 'a' and 'b' turned into 3D vectors (without the unnecessary computation).
    static double crossProductZ( const Vector2& a, const Vector2& b );

    void setX(double x);
    void setY(double y);
private:

    double _x;
    double _y;
};

} // core

#endif // #include guard
