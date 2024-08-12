#include <model/stroketools.h>

#include <exceptions/runtimeerror.h>
#include <math/curveutility.h>
#include <math/interpcubic.h>
#include <model/curvesforward.h>
#include <model/curveback.h>
#include <model/interval.h>
#include <model/polyline.h>
#include <model/posback.h>
#include <model/stroke.h>

#include <utility/bspline2utility.h>
#include <utility/casts.h>
#include <utility/mathutility.h>
#include <utility/twodarray.h>

namespace core {
namespace model {

using SharedConstStrokesVector = std::vector< std::shared_ptr< const Stroke > >;
using SharedConstStrokesSet = std::set< std::shared_ptr< const Stroke > >;

namespace {

/// Make 'aCopy' and 'bCopy' into versions of 'a' and 'b' that have the same
/// degree and knot vectors as each other (so that their control points correspond).
void makeSameDegreeAndNumControl(
    const Curve& a,
    const Curve& b,
    UniqueCurve& aCopy,
    UniqueCurve& bCopy )
{
    aCopy = a.clone();
    bCopy = b.clone();

    if( aCopy->degree() < bCopy->degree() ) {
        aCopy->degreeElevate( bCopy->degree() );
    } else if( bCopy->degree() < aCopy->degree() ) {
        bCopy->degreeElevate( aCopy->degree() );
    }
    BSpline2Utility::unionKnotVectors( *aCopy, *bCopy );
}

} // unnamed

std::unique_ptr< Stroke > setWidthCurve( const Stroke& stroke, UniqueCurve&& widthCurve )
{
    auto toReturn = std::make_unique< Stroke >( std::move( widthCurve ) );
    toReturn->setCurve( stroke.curve() );
    return toReturn;
}

UniqueCurve linearWidthCurve( double widthStart, double widthEnd )
{
    return Curve::lineSeg(
        Vector2( 0.0, widthStart ),
        Vector2( 0.0, widthEnd ) );
}

UniqueCurve multiplyWidthCurves( const Curve& a, const Curve& b )
{
    UniqueCurve aCopy, bCopy;
    makeSameDegreeAndNumControl( a, b, aCopy, bCopy );

    // The control points should have the same size now.
    const auto& aControl = aCopy->controlPoints();
    const auto& bControl = bCopy->controlPoints();
    Polyline newControl( aControl.size() );
    for( size_t i = 0; i < aControl.size(); i++ ) {
        newControl[ i ] = Vector2(
            aControl[ i ].x(),
            aControl[ i ].y() * bControl[ i ].y() );
    }
    return Curve::spline( aCopy->degree(), newControl, aCopy->internalKnots() );
}

UniqueStroke circleStroke( const model::Pos& center, double rad, double strokeWidth )
{
    auto toReturn = std::make_unique< Stroke >( strokeWidth );
    auto curve = math::circleCurve( center, rad );
    toReturn->setCurve( std::move( curve ) );
    return toReturn;
}

UniqueStroke lineSegStroke( const Vector2& posA, const Vector2& posB, double width )
{
    auto toReturn = std::make_unique< Stroke >( width );
    toReturn->setCurve( Curve::lineSeg( posA, posB ) );
    return toReturn;
}

UniqueCurve constWidthCurve( double width )
{
    return Curve::spline( 1, { Vector2( 0.0, width ), Vector2( 1.0, width ) } );
}

void multiplyWidthCurve( Curve& width, double factor )
{
    auto control = width.controlPoints();
    const auto knots = width.internalKnots();
    for( auto& p : control ) {
        p.setY( p.y() * factor );
    }
    width = Curve( width.degree(), control, knots );
}

std::unique_ptr< Stroke > multiplyStrokeWidth( const Stroke& stroke, const Curve& widthCurve )
{
    auto oldWidthCurve = stroke.widthCurve().clone();
    auto combinedWidthCurve = multiplyWidthCurves( *oldWidthCurve, widthCurve );
    return setWidthCurve( stroke, std::move( combinedWidthCurve ) );
}

std::unique_ptr< Stroke > multiplyStrokeWidth( const Stroke& stroke, double factor )
{
    auto widthCurve = stroke.widthCurve().clone();
    multiplyWidthCurve( *widthCurve, factor );
    return setWidthCurve( stroke, std::move( widthCurve ) );
}

UniqueCurve stitchC0WidthCurve(
    const std::vector< const model::Curve* >& parts,
    const std::vector< double >& tWeights )
{
    // All the parts are already G1+ w.r.t. Y; now move their X values around so they are C0 w.r.t. X. This doesn't
    // really matter for functionality (only Ys matter for width curves) but... cleaner splines in debugger?

    double totalTWeight = 0.0;
    for( const auto& tWeight : tWeights ) {
        totalTWeight += tWeight;
    }

    // Each curve's X interval ends with a location in [0,1] according to 'tWeights'.
    double weightSum = 0.0;
    std::vector< UniqueCurve > partsXShifted( parts.size() );
    double xStart = 0.0;
    for( size_t i = 0; i < parts.size(); i++ ) {
        weightSum += tWeights[ i ];
        const double xEnd = weightSum / totalTWeight;

        auto partShifted = setXIntervalForWidthCurve( *parts[ i ], xStart, xEnd );

        const auto& control = partShifted->controlPoints();
        if( i > 0 && control.size() == 2 ) {
            // If you stitch a taper-to-zero stroke to a taper-to-zero stroke, the
            // addition will be width-0 all the way unless you give its control points more play first.
            model::Polyline newControl{
                control[ 0 ],
                Vector2::lerp( control[ 0 ], control[ 1 ], 0.1 ),
                control[ 1 ] };
            partShifted = Curve::spline( partShifted->degree(), newControl );
        }

        partsXShifted[ i ] = std::move( partShifted );

        xStart = xEnd;
    }
    return BSpline2Utility::stitchC0Spline( uniquesToConstRaws( partsXShifted ), tWeights );
}

UniqueCurve setXIntervalForWidthCurve(
    const model::Curve& original, double xStart, double xEnd )
{
    const BoundingBoxd bounds = original.boundingBox();
    const double originalWidth = bounds.widthExclusive();
    if( mathUtility::closeEnoughToZero( originalWidth ) ) {
        return std::make_unique< model::Curve >( original );
    } else {
        std::vector< Vector2 > control = original.controlPoints();
        for( auto& p : control ) {
            p.setX( xStart + ( ( p.x() - bounds.xMin() ) / originalWidth ) * ( xEnd - xStart ) );
        }
        return model::Curve::createFromControlPointsAndKnots(
            original.degree(),
            control,
            original.internalKnots() );
    }
}

std::vector< double > partWeightsForC0Stitch( const std::vector< const Stroke* >& strokes )
{
    if( strokes.size() == 0 ) {
        return {};
    }
    std::vector< double > partLengths( strokes.size() );
    double totalLength = 0.0;

    for( size_t i = 0; i < strokes.size(); i++ ) {
        const auto* const stroke = strokes[ i ];
        const double partLength = stroke->curve().cachedLength( Curve::defaultLengthPrecision );
        partLengths[ i ] = partLength;
        totalLength += partLength;
    }

    if( mathUtility::closeEnoughToZero( totalLength ) ) {
        return std::vector< double >( strokes.size(), 1.0 / static_cast< double >( strokes.size() ) );
    }

    for( double& partLength : partLengths ) {
        partLength /= totalLength;
    }

    return partLengths;
}

UniqueStroke stitchC0Strokes(
    const std::vector< const Stroke* >& strokes,
    bool loop,
    const std::vector< double >& partWeights,
    std::vector< double >* storePartEndT )
{
    if( strokes.size() == 0 ) {
        return nullptr;
    }

    std::vector< const model::Curve* > spatialCurveParts( strokes.size() );
    std::vector< UniqueCurve > widthCurveParts( strokes.size() );

    for( size_t i = 0; i < strokes.size(); i++ ) {
        const auto* const stroke = strokes[ i ];
        spatialCurveParts[ i ] = &stroke->curve();
        widthCurveParts[ i ] = stroke->widthCurve().clone();
    }

    auto stitchedPath = BSpline2Utility::stitchC0Spline( spatialCurveParts, partWeights, loop, storePartEndT );
    auto stitchedWidth = stitchC0WidthCurve( uniquesToConstRaws( widthCurveParts ), partWeights );

    auto compositeStroke = std::make_unique< model::Stroke >( std::move( stitchedWidth ) );
    compositeStroke->setCurve( std::move( stitchedPath ) );
    return compositeStroke;
}

UniqueStroke stitchC0Strokes( const Stroke& a, const Stroke& b, double* stitchT )
{
    const std::vector< const Stroke* > strokes{ &a, &b };
    if( stitchT ) {
        std::vector< double > stitchTVec;
        auto ret = stitchC0Strokes( strokes, false, &stitchTVec );
        *stitchT = stitchTVec.front();
        return ret;
    } else {
        return stitchC0Strokes( strokes, false );
    }
}

UniqueStroke stitchC0Strokes(
    const std::vector< const Stroke* >& strokes, bool loop, std::vector< double >* storePartEndT )
{
    return stitchC0Strokes(
        strokes,
        loop,
        partWeightsForC0Stitch( strokes ),
        storePartEndT );
}

UniqueStroke stitchC0Strokes(
    const std::vector< UniqueStroke >& strokes,
    bool loop,
    std::vector< double >* storePartEndT )
{
    std::vector< const Stroke* > raw( strokes.size() );
    for( size_t i = 0; i < strokes.size(); i++ ) {
        raw[ i ] = strokes[ i ].get();
    }
    return stitchC0Strokes( raw, loop, storePartEndT );
}

UniqueStroke simpleSegStroke( const model::Pos& start, const model::Pos& end, double width )
{
    auto curve = Curve::lineSeg( start, end );
    auto wCurve = constWidthCurve( width );
    return strokeFromPosAndWidth( std::move( curve ), std::move( wCurve ) );
}

bool isSimpleSegStroke( const Stroke& s )
{
    const auto& posCurve = s.curve();
    if( posCurve.degree() > 1 || posCurve.controlPoints().size() > 2 ) {
        return false;
    }

    const auto& wCurve = s.widthCurve();
    const double minVarAllowed = 1e-5;
    const auto& control = wCurve.controlPoints();
    for( size_t i = 1; i < control.size(); i++ ) {
        if( std::abs( control[ i ].y() - control[ 0 ].y() ) > minVarAllowed ) {
            return false;
        }
    }
    return true;
}

std::unique_ptr< Stroke > strokeFromPosAndWidth( UniqueCurve&& posToOwn, UniqueCurve&& widthToOwn )
{
    auto ret = std::make_unique< Stroke >( std::move( widthToOwn ) );
    ret->setCurve( std::move( posToOwn ) );
    return ret;
}

bool strokesAreApproxC0( const std::vector< const Stroke* >& parts, bool closed, double maxErrorDist )
{
    RawConstCurves curves( parts.size() );
    for( size_t i = 0; i < parts.size(); i++ ) {
        curves[ i ] = &parts[ i ]->curve();
    }
    return math::curvesAreApproxC0( curves, closed, maxErrorDist );
}

bool strokesAreApproxC0( const std::vector< UniqueStroke >& parts, bool closed, double maxErrorDist )
{
    const auto raws = uniquesToConstRaws( parts );
    return strokesAreApproxC0( raws, closed, maxErrorDist );
}

UniqueStroke taperStrokeEndpoints( const Stroke& stroke, boost::optional< double > tStart, boost::optional< double > tEnd )
{
    /// Let X be T in [0,1] and Y by a width multiplier in [0,1]
    using XY = math::InterpCubic::XY;

    math::InterpCubic::XYs xy;
    if( tStart && tEnd ) {
        if( *tStart < *tEnd ) {
            if( *tStart > 0. ) {
                xy.push_back( XY{ 0., 0. } );
            }
            xy.push_back( XY{ *tStart, 1. } );
            xy.push_back( XY{ *tEnd, 1. } );
            if( tEnd < 1. ) {
                xy.push_back( XY{ 1., 0. } );
            }
        } else {
            const auto tMid = ( *tStart + *tEnd ) / 2.;
            xy = {
                XY{ 0., 0. },
                XY{ tMid, 1. },
                XY{ 1., 0. }
            };
        }
    } else if( tStart ) {
        if( tStart > 0. ) {
            xy.push_back( XY{ 0., 0. } );
        }
        xy.push_back( XY{ *tStart, 1. } );
        xy.push_back( XY{ 1., 1. } );
    } else if( tEnd ) {
        xy.push_back( XY{ 0., 1. } );
        xy.push_back( XY{ *tEnd, 1. } );
        if( tEnd < 1. ) {
            xy.push_back( XY{ 1., 0. } );
        }
    } else {
        THROW_UNEXPECTED( "At least one of the ends should be tapered" );
    }

    math::InterpCubic interp( xy );
    const auto& widthCurveMult = interp.spline();
    return multiplyStrokeWidth( stroke, widthCurveMult );
}

} // model
} // core
