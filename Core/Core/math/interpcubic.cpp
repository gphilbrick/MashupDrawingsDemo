#include <math/interpcubic.h>

#include <exceptions/runtimeerror.h>
#include <model/curveback.h>

#include <utility/bspline2utility.h>
#include <utility/casts.h>
#include <utility/mathutility.h>

namespace core {
namespace math {

InterpCubic::InterpCubic( const XYs& xy )
{
    _xy = filterEqualX( xy );
    if( _xy.size() < 2 ) {
        THROW_RUNTIME( "Need more unique-X coordinates." );
    }
    const size_t numCubics = _xy.size() - 1;
    std::vector< double > curveWeights( numCubics );
    std::vector< model::UniqueCurve > cubics( numCubics );

    for( size_t i = 0; i < numCubics; i++ ) {
        const auto& xyA = _xy[ i ];
        const auto& xyB = _xy[ i + 1 ];
        const auto xLen = ( xyB.x() - xyA.x() );
        curveWeights[ i ] = xLen;

        constexpr double leg = 0.25;
        const XYs bezierControl
        {
            xyA,
            xyA + XY( leg * xLen, 0. ),
            xyB + XY( -leg * xLen, 0. ),
            xyB
        };
        cubics[ i ] = model::Curve::spline( 3, bezierControl );
    }
    _spline = BSpline2Utility::stitchC0Spline( uniquesToConstRaws( cubics ), curveWeights, false );
}

double InterpCubic::yFromX( double x ) const
{
    double f = 0.;
    const auto denom = _xy.back().x() - _xy.front().x();
    if( !mathUtility::closeEnoughToZero( denom ) ) {
        f = std::clamp( ( x - _xy.front().x() ) / denom, 0., 1. );
    }
    return yFromF( f );
}

double InterpCubic::yFromF( double f ) const
{
    return _spline->position( f ).y();
}

InterpCubic::XYs InterpCubic::filterEqualX( const XYs& xy )
{
    XYs ret;
    ret.push_back( xy.front() );

    for( size_t i = 1; i < xy.size(); i++ ) {
        const auto& next = xy[ i ];
        if( next.x() < ret.back().x() ) {
            THROW_RUNTIME( "Decreasing X given" );
        }
        if( mathUtility::closeEnough( ret.back().x(), next.x() ) ) {
            ret.back() = next;
        } else {
            ret.push_back( next );
        }
    }
    return ret;
}

const model::Curve& InterpCubic::spline() const
{
    return *_spline;
}

} // math
} // core
