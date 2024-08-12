#include <math/curveutility.h>

#include <exceptions/runtimeerror.h>
#include <model/curveback.h>
#include <model/interval.h>
#include <model/lineback.h>

#include <utility/bspline2utility.h>
#include <utility/casts.h>
#include <utility/ellipse.h>
#include <utility/mathutility.h>

#include <clipper2/clipper.h>

#include <algorithm>

namespace core {
namespace math {

using Pos = model::Pos;
using Curve = model::Curve;
using Seg = model::Seg;
using UniqueCurve = model::UniqueCurve;
using UniqueCurves = model::UniqueCurves;
NumBeziersPolylineLength numBezierPolylineLengthDefault = NumBeziersPolylineLength{ 10 };

namespace {

/// Return a joint curve that starts at end of 'a' (G1 with 'a'), passes through 'x', and ends
/// at start of 'b' (G1 with 'b'). Use a compound curve made of 2 degree-2 Beziers. Return
/// nullptr in event of failure (will only succeed in relatively ideal scenarios).
model::UniqueCurve smoothJoint_twoDeg2(
    const model::Curve& a,
    const model::Pos& x,
    const model::Curve& b )
{
    /// A position on some ray (<0 means before the start of the ray).
    using RayCoord = double;

    /// An 'ACoord' interval or a 'BCoord' interval.
    using RayInterval = BoundingIntervald;

    struct Ray
    {
        RayCoord coordOnRay( const Pos& onRay ) const
        {
            const auto startToOn = onRay - start;
            const auto len = startToOn.length();
            return Pos::dot( startToOn, dir ) > 0. ? len : -len;
        }

        Pos onRay( RayCoord coord ) const
        {
            return start + dir * coord;
        }

        Pos projToRay( const Pos& p ) const
        {
            return mathUtility::projectPointOntoLine( p, start, start + dir );
        }

        Seg asLine() const
        {
            return { start, start + dir };
        }

        Pos start;
        /// normalized
        Pos dir;
    };

    // 'ACoord's are defined w.r.t. this ray.
    Ray aRay;
    aRay.start = a.endPosition();
    aRay.dir = a.derivative( 1. );
    aRay.dir.normalize();

    // 'BCoord's are defined w.r.t. this ray.
    Ray bRay;
    bRay.start = b.startPosition();
    bRay.dir = b.derivative( 0. ) * -1.;
    bRay.dir.normalize();

    // Start of 'aRay' to 'x' projected onto 'aRay'
    const auto aCoordForX = aRay.coordOnRay( aRay.projToRay( x ) );
    if( aCoordForX <= 0. ) {
        // 'x' is behind start of 'aRay'.
        return nullptr;
    }
    const RayInterval aStartToX{ 0., aCoordForX };

    // Start of 'bRay' to 'x' projected onto 'bRay'.
    const auto bCoordForX = bRay.coordOnRay( bRay.projToRay( x ) );
    if( bCoordForX <= 0. ) {
        // 'x' is behind start of 'bRay'.
        return nullptr;
    }
    const RayInterval bStartToX{ 0., bCoordForX };

    // "Allowed" means that we're allowed to place control points
    // on these subranges.

    // [0,1] value saying how far in front of a ray we need to be before
    // we're allowed to place control points. Smaller means avoiding ugly
    // sharp corners in results, but larger means risking failure for this
    // method.
    const double forbidAmount = 0.3;
    const RayInterval aAllowed( aStartToX.lerp( forbidAmount ), aStartToX.max() );
    const RayInterval bAllowed( bStartToX.lerp( forbidAmount ), bStartToX.max() );

    /// Pass a line from the indicated position on 'ray' through 'x' and return the 'otherRay'
    /// coordinate where said line intersects 'otherRay' (or boost::none if no intersection).
    const auto coordOnOtherRay = [ &x ]( const Ray& ray, double rayCoord, const Ray& otherRay )
        -> boost::optional< RayCoord >
    {
        const Seg lineThroughX{ ray.onRay( rayCoord ), x };
        bool valid = false;
        const auto res = mathUtility::lineIntersection( lineThroughX, otherRay.asLine(), valid );
        if( valid ) {
            return otherRay.coordOnRay( res );
        } else {
            return boost::none;
        }
    };

    /// Project 'interval' from 'ray' to 'otherRay' by shooting lines through 'x'. Then return
    /// the intersection of that projected interval on 'otherRay' with 'otherInterval' and return
    /// it, or return boost::none if there is no intersection.
    const auto survivingOtherInterval =
        [ & ]( const Ray& ray, const Ray& rayOther, const RayInterval& interval, const RayInterval& otherInterval )
        -> boost::optional< RayInterval >
    {
        const auto minProj = coordOnOtherRay( ray, interval.min(), rayOther );
        if( !minProj ) {
            return boost::none;
        }
        const auto maxProj = coordOnOtherRay( ray, interval.max(), rayOther );
        if( !maxProj ) {
            return boost::none;
        }

        const RayInterval intervalProj{ *minProj, *maxProj };
        return RayInterval::intersection( otherInterval, intervalProj );
    };

    const auto aSurviving = survivingOtherInterval( bRay, aRay, bAllowed, aAllowed );
    if( !aSurviving ) {
        return nullptr;
    }
    const auto bSurviving = survivingOtherInterval( aRay, bRay, aAllowed, bAllowed );
    if( !bSurviving ) {
        return nullptr;
    }

    const auto chosenACoord = aSurviving->midpoint();
    const auto chosenBCoord = coordOnOtherRay( aRay, chosenACoord, bRay );
    if( !chosenBCoord ) {
        // ltwarning: Saw this happen and not spending time right now to figure out if
        // this is a bug or a reasonable fail case.
        return nullptr;
    }
    if( !bAllowed.contains( *chosenBCoord ) ) {
        return nullptr;
    }

    const auto chosenAPos = aRay.onRay( chosenACoord );
    const auto chosenBPos = bRay.onRay( *chosenBCoord );

    // SAFETY CHECKS
    {
        // This method is only really supposed to work in cases where 'x' is between
        // the two rays. We know that 'x', 'chosenAPos', and 'chosenBPos' define a line, so
        // check that 'x' is between the other two.
        if( Pos::dot( chosenAPos - x, chosenBPos - x ) >= 0. ) {
            return nullptr;
        }

        // It's possible to get a technically valid joint that looks discontinuous at
        // 'x' b/c 'x' is super close to one or both of the interior control points.
        const auto safetyDist = ( aRay.start - bRay.start ).length() * 0.1;
        if( ( chosenAPos - x ).length() < safetyDist ) {
            return nullptr;
        }
        if( ( chosenBPos - x ).length() < safetyDist ) {
            return nullptr;
        }
    }

    const auto bezier_a_to_x = Curve::spline(
        2,
        { aRay.start, chosenAPos, x } );
    const auto bezier_x_to_b = Curve::spline(
        2,
        { x, chosenBPos, bRay.start } );
    return BSpline2Utility::stitchC0Spline(
        { bezier_a_to_x.get(), bezier_x_to_b.get() },
        Curve::defaultLengthPrecision );
}

} // unnamed

NumBeziersPolylineLength::NumBeziersPolylineLength( size_t pointsPerCurveVal ) : pointsPerCurve( pointsPerCurveVal )
{
}

size_t NumBeziersPolylineLength::operator()( const model::Curve& curve ) const
{
    return curve.numBezierCurves( false ) * pointsPerCurve;
}

model::UniqueCurve transformCurveForGrid(
    const model::Curve& c,
    const Vector2& gridTopLeft,
    double cellWidth )
{
    return transformCurveForGrid( c, gridTopLeft, cellWidth, cellWidth );
}

model::UniqueCurve transformCurveForGrid( const model::Curve& c, const Vector2& gridTopLeft, double cellWidth, double cellHeight )
{
    auto copy = c.clone();
    copy->transform( [ & ]( const Vector2& p )
    {
        return Vector2
        {
            ( p.x() - gridTopLeft.x() ) / cellWidth,
            ( p.y() - gridTopLeft.y() ) / cellHeight
        };
    } );
    return copy;
}

boost::optional< model::Interval > trimCurveEndsInterval( const model::Curve& toTrim,
    boost::optional< Vector2 > startTrimDir,
    boost::optional< Vector2 > endTrimDir,
    double startTrimDist,
    double endTrimDist,
    const IntersectionParameters& params )
{
    const auto lineSegLength = toTrim.boundingBox().maxDim() * 2.0;

    const auto findEndpointT = [ & ]( bool startOrEnd, double trimDist, const Vector2& trimDir ) -> double
    {
        // Create a line segment to collide with 'toTrim'.
        const auto toTrimEndpoint = startOrEnd ? toTrim.startPosition() : toTrim.endPosition();
        auto endpointDir = startOrEnd ? toTrim.derivative( 0.0 ) * -1.0 : toTrim.derivative( 1.0 );
        auto dirPerp = trimDir;
        dirPerp.turnPerpendicular();
        dirPerp.normalize();

        auto trimDirNorm = trimDir;
        trimDirNorm.normalize();

        const auto dot = Vector2::dot( dirPerp, endpointDir );
        const auto lineOrigin = dot < 0 ? toTrimEndpoint + dirPerp * trimDist : toTrimEndpoint - dirPerp * trimDist;

        // Approximate a line by just making a line segment long enough.
        LineSegment lineSeg( lineOrigin + trimDirNorm * lineSegLength, lineOrigin - trimDirNorm * lineSegLength );
        CurveCurveIntersections hits;
        BSpline2Utility::lineSegmentIntersections( toTrim, lineSeg, hits, params );
        if( hits.size() > 0 ) {

            // Sort the hits by the T value in A.
            class SortFunctor
            {
            public:
                bool operator()( CurveCurveIntersection& a, CurveCurveIntersection& b ) const
                {
                    return a.tIntervalA.midpoint() < b.tIntervalA.midpoint();
                }
            };
            std::sort( hits.begin(), hits.end(), SortFunctor{} );

            const auto& myHit = startOrEnd ? hits.front() : hits.back();
            return myHit.tIntervalA.midpoint();
        } else {
            return startOrEnd ? 0.0 : 1.0;
        }
    };

    const auto tStart = startTrimDir.is_initialized() ? findEndpointT( true, startTrimDist, startTrimDir.value() ) : 0.0;
    const auto tEnd = endTrimDir.is_initialized() ? findEndpointT( false, endTrimDist, endTrimDir.value() ) : 1.0;

    if( tStart >= tEnd ) {
        return boost::none;
    } else {
        return model::Interval{ tStart, tEnd };
    }
}

model::UniqueCurve moveCurveEndpoint( const model::Curve& c, const Vector2& newEndpoint, bool startOrEnd )
{
    auto control = c.controlPoints();
    control[ startOrEnd ? 0 : control.size() - 1 ] = newEndpoint;
    return model::Curve::spline( c.degree(), control, c.internalKnots() );
}

model::UniqueCurve moveCurveEndpoints( const model::Curve& c, const Vector2& newStart, const Vector2& newEnd )
{
    auto control = c.controlPoints();
    control.front() = newStart;
    control.back() = newEnd;
    return model::Curve::spline( c.degree(), control, c.internalKnots() );
}

model::UniqueCurve moveCurveEndpoints( const model::Curve& c,
                                       const boost::optional< Vector2 >& newStart,
                                       const boost::optional< Vector2 >& newEnd )
{
    auto control = c.controlPoints();
    if( newStart )  {
        control.front() = newStart.value();
    }
    if( newEnd ) {
        control.back() = newEnd.value();
    }
    return model::Curve::spline( c.degree(), control, c.internalKnots() );
}

model::UniqueCurve smoothJoint( const model::Pos& aPos,
                                const model::Pos& aDir_,
                                const model::Pos& bPos,
                                const model::Pos& bDir_ )
{
    auto aDir = aDir_;
    aDir.normalize();

    auto bDir = bDir_ * -1.;
    bDir.normalize();

    const auto heuristicDist = ( bPos - aPos ).length();
    const auto legDist = heuristicDist * 0.3;

    // Do the two "legs" intersect?
    const auto aPos2 = aPos + aDir * legDist;
    const auto bPos2 = bPos + bDir * legDist;

    model::Polyline posControl;

    model::Pos hit;
    if( mathUtility::segmentsIntersect( aPos, aPos2, bPos, bPos2, hit ) ) {
        posControl = { aPos, hit, bPos };
    } else {
        posControl = { aPos, aPos2, bPos2, bPos };
    }

    auto posCurve = model::Curve::spline(
        static_cast< int >( posControl.size() - 1 ),
        posControl );
    return posCurve;
}

model::UniqueCurve smoothJoint(
    const model::Curve& a,
    const model::Pos& x,
    const model::Curve& b )
{
    auto coolRes = smoothJoint_twoDeg2( a, x, b );
    if( coolRes ) {
        return coolRes;
    }

    // Fall back on less-cool way using 2 degree-3 Beziers.

    auto aToX = x - a.endPosition();
    aToX.normalize();

    auto xToB = b.startPosition() - x;
    xToB.normalize();

    auto dirThroughX = aToX + xToB;

    auto joint1 = smoothJoint( a.endPosition(), a.derivative( 1. ), x, dirThroughX );
    if( !joint1 ) {
        return nullptr;
    }
    auto joint2 = smoothJoint( x, dirThroughX, b.startPosition(), b.derivative( 0. ) );
    if( !joint2 ) {
        return nullptr;
    }
    return BSpline2Utility::stitchC0Spline(
        { joint1.get(), joint2.get() },
        model::Curve::defaultLengthPrecision );
}

model::UniqueCurve smoothJoint( const model::Curve& a, const model::Curve& b )
{
    return smoothJoint( a.endPosition(), a.derivative( 1. ), b.startPosition(), b.derivative( 0. ) );
}

double eraseCircleT( const model::Curve& curve, double rad, bool start, size_t numSteps )
{
    const auto center = start ? curve.startPosition() : curve.endPosition();
    const auto& otherEnd = start ? curve.endPosition() : curve.startPosition();
    if( ( otherEnd - center ).length() <= rad ) {
        return start ? 1. : 0.;
    }

    // Do a binary search, assuming that 'curve' is split into
    // an outside-the-circle half and an inside-the-circle half.
    double tOutside = start ? 1. : 0.;
    double tInside = start ? 0. : 1.;

    for( size_t i = 0; i < numSteps; i++ ) {
        const auto tMid = ( tOutside + tInside ) / 2.;
        const auto pos = curve.position( tMid );
        if( ( pos - center ).length() <= rad ) {
            tInside = tMid;
        } else {
            tOutside = tMid;
        }
    }
    const auto t_curve = ( tOutside + tInside ) / 2.;
    return t_curve;
}

model::UniqueCurve circleCurve( const model::Pos& center, double rad )
{
    Ellipse::Parametric ellipse;
    ellipse.a = rad;
    ellipse.b = rad;
    ellipse.center = center;
    return ellipse.splineApprox();
}

model::Polylines inflatePolyline( const model::Polyline& poly, double inflateBy )
{
    // Inflate using Clipper2
    Clipper2Lib::PathsD clipperPaths;
    {
        clipperPaths.push_back( {} );
        Clipper2Lib::PathD clipperPath;
        for( const auto& p : poly ) {
            clipperPaths.back().push_back( { p.x(), p.y() } );
        }
    }

    const auto res = Clipper2Lib::InflatePaths(
        clipperPaths,
        inflateBy,
        Clipper2Lib::JoinType::Bevel,
        Clipper2Lib::EndType::Round,
        2. );

    model::Polylines inflatedPolys( res.size() );
    for( size_t i = 0; i < res.size(); i++ ) {
        const auto& clipperP = res[ i ];
        auto& convertBack = inflatedPolys[ i ];
        convertBack.resize( clipperP.size() );
        for( size_t i = 0; i < clipperP.size(); i++ ) {
            convertBack[ i ] = model::Pos{ clipperP[ i ].x, clipperP[ i ].y };
        }
        // Clipper2's output paths are implicitly closed.
    }
    return inflatedPolys;
}

model::Polyline evenResamplePolyline( const model::Polyline& poly, size_t numSamples )
{
    if( poly.size() < 2 || numSamples < 2 ) {
        THROW_UNEXPECTED;
    }

    double polyLen = 0.;
    std::vector< double > segLengths( poly.size() - 1  );
    for( size_t i = 0; i < poly.size() - 1; i++ ) {
        segLengths[ i ] = ( poly[ i + 1 ] - poly[ i ] ).length();
        polyLen += segLengths[ i ];
    }

    const auto stepDist = polyLen / static_cast< double >( numSamples - 1 );
    if( mathUtility::closeEnoughToZero( stepDist ) ) {
        THROW_UNEXPECTED;
    }

    model::Polyline ret;
    ret.push_back( poly.front() );

    const auto numSegs = segLengths.size();
    size_t segIdx = 0; // in [0, poly.size() - 1 ]
    double tillNextSample = stepDist;
    while( segIdx < numSegs ) {
        auto segLen = segLengths[ segIdx ];
        while( segLen > 0. ) {
            if( segLen >= tillNextSample ) {
                segLen -= tillNextSample;
                const auto f = 1. - segLen / segLengths[ segIdx ];
                ret.push_back( model::Pos::lerp( poly[ segIdx ], poly[ segIdx + 1 ], f ) );
                tillNextSample = stepDist;
            } else {
                tillNextSample -= segLen;
                segLen = 0.;
            }
        }
        segIdx++;
    }
    if( ret.size() == numSamples - 1 ) {
        ret.push_back( poly.back() );
    } else if( ret.size() == numSamples ) {
        ret.back() = poly.back();
    } else {
        THROW_UNEXPECTED;
    }
    return ret;
}

bool curvesAreApproxC0( const model::RawConstCurves& parts, bool closed, double maxErrorDistAllowed )
{
    if( parts.size() == 0 ) {
        THROW_UNEXPECTED;
    }
    const size_t numStitches = closed ? parts.size() : parts.size() - 1;
    for( size_t i = 0; i < numStitches; i++ ) {
        const auto& curveA = *parts[ i ];
        const auto& curveB = *parts[ ( i + 1 ) % parts.size() ];
        const auto endpointsOffset = curveA.endPosition() - curveB.startPosition();
        if( endpointsOffset.length() > maxErrorDistAllowed ) {
            return false;
        }
    }
    return true;
}

bool curvesAreApproxC0( const model::UniqueCurves& parts, bool closed, double maxErrorDistAllowed )
{
    const auto raws = uniquesToConstRaws( parts );
    return curvesAreApproxC0( raws, closed, maxErrorDistAllowed );
}

} // math
} // core
