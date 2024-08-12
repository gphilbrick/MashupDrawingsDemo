#ifndef CORE_MATH_INTERPCUPBIC_H
#define CORE_MATH_INTERPCUPBIC_H

#include <Core/model/curveforward.h>
#include <Core/model/polyline.h>

namespace core {
namespace math {

/// Takes in an increasing-X series of (X,Y) pairs and then interpolates Y from
/// a given X (or from an F value mapped to the X domain).
///
/// This interpolates Y s.t. returned Y does not go outside the corresponding
/// given Y interval.
class InterpCubic
{
public:
    using XY = model::Pos;
    using XYs = model::Polyline;

    /// The X values in 'xy' (length > 1) must be increasing (if consecutive
    /// pairs have near-equal X values, only the lattermost pair will be considered,
    /// and this will throw if said filtering yields < 2 valid pairs).
    InterpCubic( const XYs& xy );
    double yFromX( double ) const;
    /// 'f' in [0,1], which interval corresponds to the X-range given to constructor.
    double yFromF( double f ) const;
    const model::Curve& spline() const;
private:
    static XYs filterEqualX( const XYs& xy );

    XYs _xy;
    model::UniqueCurve _spline;
};

} // math
} // core

#endif // #include
