#ifndef CORE_BEZIERSFROMSPLINE_H
#define CORE_BEZIERSFROMSPLINE_H

#include <Core/utility/bspline2.h>

#include <memory>

namespace core {

/// Provides access to each of the individual Beziers curves of a 'BSpline2' 'spline', avoiding
/// unnecessary copying if the original 'spline' already represents just a single Bezier curve.
class BeziersFromSpline
{
public:
    using Spline = BSpline2;
    /// A spline that represents a single Bezier curve.
    using Bezier = BSpline2;
    using BezierHandles = std::vector< const Bezier* >;

    /// 'spline' must be unchanged through lifetime of 'this'.
    BeziersFromSpline( const Spline& spline );
    const BezierHandles& beziers() const;
    /// Return the T-start values within 'spline' of all of 'spline''s component Bezier curves.
    const std::vector< double >& tStarts() const;
private:
    BezierHandles _beziers;
    std::vector< std::unique_ptr< Bezier > > _createdBeziers;
    std::vector< double > _tStarts;
};

} // core

#endif // #include
