#ifndef CORE_ELLIPSE_H
#define CORE_ELLIPSE_H

#include <Core/utility/polarinterval.h>
#include <Core/utility/vector2.h>

#include <memory>
#include <vector>

namespace core {

class BSpline2;

class Ellipse
{
public:
    /// A definition of an ellipse in terms of the equation
    /// ax^2 + bxy + cy^2 + dx + ey + f = 0.
    struct Conic
    {
        Conic() : a( 0 ), b( 0 ), c( 0 ), d( 0 ), e( 0 ), f( 0 ) {}
        double a;
        double b;
        double c;
        double d;
        double e;
        double f;
    };

    struct Parametric
    {
        Parametric() : a( 0 ), b( 0 ), tauCounterclockwise( 0 ) {}

        /// Return an angle-based position on the ellipse (0 points ahead in the major-axis direction).
        Vector2 pos( double radians ) const;
        /// Return whether there are any NaNs or infinities in the definition.
        bool valid() const;
        /// Return the ellipse-relative polar angle of the provided point (which is not necessarily on the ellipse)
        /// relative to the ellipse's center. Store the unit-circle-space transformation of 'pos' in 'storeUnitCircle'.
        double angle( const Vector2& pos, Vector2& storeUnitCircle ) const;
        /// Return a spline of 4 cubic Beziers.
        std::unique_ptr< BSpline2 > splineApprox() const;
        /// Return an arc-approximating spline or nullptr if the angle interval is too small.
        std::unique_ptr< BSpline2 > splineApprox( const PolarInterval& angleInterval ) const;

        Vector2 center;
        double a;
        double b;
        double tauCounterclockwise;
    private:
        void unitCircleToEllipse( std::vector< Vector2 >& ) const;
    };

    static Conic conic( const Parametric& );
    static Parametric circle( const Vector2& center, double radius );
};

} // core

#endif // #include
