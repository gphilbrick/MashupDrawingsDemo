#ifndef CORE_MATH_CURVEUTILITY_H
#define CORE_MATH_CURVEUTILITY_H

#include <Core/model/curvesforward.h>
#include <Core/model/interval.h>
#include <Core/model/polyline.h>
#include <Core/model/stroke.h>

#include <functional>

namespace core {
struct IntersectionParameters;
template< typename T >
class TwoDArray;
} // core

namespace core {
namespace math {

/// A functor for getting the number of points for a polyline approximation of a curve based
/// on the number of non-degenerate component Bezier curves.
struct NumBeziersPolylineLength
{
    NumBeziersPolylineLength( size_t pointsPerCurveVal );
    size_t operator()( const model::Curve& curve ) const;
    size_t pointsPerCurve;
};

extern NumBeziersPolylineLength numBezierPolylineLengthDefault;

model::UniqueCurve transformCurveForGrid(
    const model::Curve& c,
    const core::Vector2& gridTopLeft,
    double cellWidth,
    double cellHeight );
model::UniqueCurve transformCurveForGrid(
    const model::Curve& c,
    const core::Vector2& gridTopLeft,
    double cellWidth );
	
/// Return the ordered T-interval used to create trimmed version of 'toTrim'. To understand what trimming means, imagine that
/// 'toTrim' collides with a wall at one or both ends. 'startTrimDir' and 'endTrimDir' indicate
/// the direction (not the normal) of the wall at the two collision points (boost::none means
/// no collision at that end, and consequently no trimming). The idea is to remove enough from
/// 'toTrim' at each end to make its new endpoint 'startTrimDist' or 'endTrimDist' away from
/// each wall, respectively. Return boost::none if the trim would completely erase the curve.
/// The two trim dirs need not be normalized.
boost::optional< model::Interval > trimCurveEndsInterval(
    const model::Curve& toTrim,
    boost::optional< core::Vector2 > startTrimDir,
    boost::optional< core::Vector2 > endTrimDir,
    double startTrimDist,
    double endTrimDist,
    const core::IntersectionParameters& );

model::UniqueCurve moveCurveEndpoint( const model::Curve& c, const core::Vector2& newEndpoint, bool startOrEnd );
model::UniqueCurve moveCurveEndpoints( const model::Curve& c, const core::Vector2& newStart, const core::Vector2& newEnd );
/// At least one of 'newEnd' and 'newStart' should be set.
model::UniqueCurve moveCurveEndpoints( const model::Curve& c,
                                       const boost::optional< core::Vector2 >& newStart,
                                       const boost::optional< core::Vector2 >& newEnd );

/// Produce a joining curve connecting the end of 'a' with start of 'b', being G1
/// with both curves at said endpoints.
model::UniqueCurve smoothJoint( const model::Curve& a, const model::Curve& b );
/// Produce a joining curve that leaves the end of 'a' (G1 with it), passes through 'visitBetween', then
/// terminates at the start of 'b' (G1 with it).
model::UniqueCurve smoothJoint( const model::Curve& a, const model::Pos& visitBetween, const model::Curve& b );
model::UniqueCurve smoothJoint( const model::Pos& aEndPos,
                                const model::Pos& aEndDerivative,
                                const model::Pos& bStartPos,
                                const model::Pos& bStartDerivative );

/// If 'start' is true, return the T where 'curve' leaves the 'rad'-circle placed at its start,
///     and return 1. if 'curve' never does leave.
/// If 'start' is false, return the T where 'curve' enters the 'rad'-circle placed at its end,
///     and return 0. if 'curve' is inside all along.
double eraseCircleT( const model::Curve& curve, double rad, bool start, size_t numBinSearchSteps = 20 );

model::UniqueCurve circleCurve( const model::Pos&, double r );

/// 'inflateBy' > 0
/// Return polygons are all implicitly closed (front/back are not duplicated but are understood to be connected).
model::Polylines inflatePolyline( const model::Polyline& poly, double inflateBy );

/// Make a spatially even resampling of 'poly'. 'numSamples' >= 2
model::Polyline evenResamplePolyline( const model::Polyline& poly, size_t numSamples );

/// Given 'parts' (size > 0) meant to be stitched into a single (possibly 'closed') curve, return
/// whether adjoining endpoints that ought in principle to be equal differ spatially by
/// no more than 'maxErrorDistAllowed'.
bool curvesAreApproxC0( const model::RawConstCurves& parts, bool closed, double maxErrorDistAllowed );
bool curvesAreApproxC0( const model::UniqueCurves& parts, bool closed, double maxErrorDistAllowed );

} // math
} // core

#endif // #include
