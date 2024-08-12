#include <utility/curvesegment.h>

#include <utility/bspline2.h>

namespace core {

CurveSegment::CurveSegment( bool aOrBVal ) :
    aOrB( aOrBVal )
{
}

CurveSegment::CurveSegment()
    : aOrB( false )
{
}

void CurveSegment::wholeCurveToSegment( const BSpline2& curve, bool aOrB, CurveSegment& store )
{
    store.aOrB = aOrB;
    store.bounds = curve.boundingBox();
    store.control = curve.controlPoints();
    store.tInterval = BoundingIntervald( 0.0, 1.0 );
}

} // core
