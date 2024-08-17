#include <PrintCurves/curvespostscript.h>

#include <PrintCurves/miteredcurve.h>
#include <PrintCurves/strokeproperties.h>

#include <Core/utility/bspline2.h>
#include <Core/utility/bspline2utility.h>
#include <Core/utility/casts.h>
#include <Core/utility/linesegment.h>
#include <Core/utility/mathutility.h>
#include <Core/utility/vector3.h>

#include <boost/math/constants/constants.hpp>
#include <boost/optional.hpp>

#include <iomanip>
#include <functional>

namespace printCurves {

namespace {

using Curve = CurvesPostScript::Curve;
using Vec2 = core::Vector2;
using RGB = CurvesPostScript::RGB;
using Vec2Vec = std::vector< Vec2 >;

void setDefaultPrecision( std::stringstream& str )
{
    // ltwarning: This assumes canvas dimensions are on the order of 2000x1000. If they get smaller, export
    // will become too lossy.
    const int placesAfterDecimal = 1;
    str << std::fixed << std::setprecision( placesAfterDecimal );
}

void setColorPrecision( std::stringstream& str )
{
    // We only need to capture 0-255 in the form of 0.xxxx.
    str << std::defaultfloat << std::setprecision( 4 );
}

void printPoint( const core::Vector2& point, std::stringstream& str )
{
    str << point.x() << " " << point.y();
}

/// Given normals 'a' and 'b', return a vector of interpolating normals.
std::vector< core::Vector2 > interpolatingDirs( const core::Vector2& a, const core::Vector2& b )
{
    core::Vector2 interp = a + b;
    interp.normalize();
    return { interp };
}

core::Vector2 keepWithinRangeOf( const core::Vector2& keepCloseTo, const core::Vector2& keepClose, double maxDist )
{
    auto vec = keepClose - keepCloseTo;
    const auto curDist = vec.length();
    if( curDist > maxDist ) {
        vec.normalize();
        return keepCloseTo + vec * maxDist;
    } else {
        return keepClose;
    }
}

/// Return a version of 'toAlign' where the first/last point lies on the line defined by 'onWall' and 'wallNormal'. If 'alignAtEndOrStart'
/// is true, apply to end, otherwise apply to start. 'toAlign' must have at least two points. When moving a point from 'toAlign' to a point
/// on the "wall," move it along the line defined by 'shiftDir', which need not be normalized. 'strokeWidth' indicates the width of
/// the stroke that 'toAlign' is presumably taken from one side or the other of--use it to deal with osculation problems.
Vec2Vec alignSamplesAgainstWall(
    const Vec2Vec& toAlign,
    const Vec2& onWall,
    const Vec2& wallNormal,
    bool alignAtEndOrStart,
    const Vec2& shiftDir,
    double strokeWidth )
{
    Vec2Vec toReturn;
    if( toAlign.size() < 2 ) {
        return toAlign;
    }

    const int startIdx = alignAtEndOrStart ? static_cast< int >( toAlign.size() - 1 ) : 0;
    const int endIdxExclusive = alignAtEndOrStart ? -1 : static_cast< int >( toAlign.size() );
    const int idxInc = alignAtEndOrStart ? -1 : 1;

    auto wallNormalPerp = wallNormal;
    wallNormalPerp.turnPerpendicular();

    const core::LineSegment wallLineSeg{ onWall, onWall + wallNormalPerp };

    // This heuristic is us compensating for the fact that we're approximating the wall as a line. It may, in fact, curve.
    // If the curve represented by 'toAlign' is close to parallel at its endpoint with our approximate wall, then our clamped-to-wall points
    // can end up being very far away from 'toAlign', causing a very visible artifact (think of the long line you saw at the end of
    // a streamline in Twee).
    const double maxStretchDistAllowed = strokeWidth * 2.0; // The name isn't really communicative, I admit.

    bool foundPointOnRightSide = false;
    for( int idx = startIdx; idx != endIdxExclusive; idx += idxInc ) {
        const auto& p = toAlign[ idx ];
        if( foundPointOnRightSide ) {
            toReturn.push_back( p );
        } else {
            // Is 'p' on the correct side of the wall?
            Vec2 onWallToP = p - onWall;
            onWallToP.normalize();
            if( Vec2::dot( wallNormal, onWallToP ) >= 0 ) {
                // Include the point between the current one (on right side of wall) and the
                // previous one (on wrong side of wall) that lies right on the line.
                if( idx != startIdx ) {
                    bool valid = false;
                    auto onLine = core::mathUtility::lineIntersection( wallLineSeg, core::LineSegment( p, p + shiftDir ), valid );
                    if( valid ) {                        
                        // I think there will be problems if 'onLine' is too close to 'p'.
                        if( !core::mathUtility::closeEnoughToZero( ( onLine - p ).length() ) ) {
                            onLine = keepWithinRangeOf( onWall, onLine, maxStretchDistAllowed );
                            toReturn.push_back( onLine );
                        }
                    }
                } else {
                    // The first point we've encountered is already on the correct side of the line, which means we
                    // need to add another, effectively extrapolating from the end of 'toAlign'.
                    bool valid = false;
                    auto onLine = core::mathUtility::lineIntersection( wallLineSeg, core::LineSegment( p, p + shiftDir ), valid );
                    if( valid ) {
                        // I think there will be problems if 'onLine' is too close to 'p'.
                        if( !core::mathUtility::closeEnoughToZero( ( onLine - p ).length() ) ) {
                            onLine = keepWithinRangeOf( onWall, onLine, maxStretchDistAllowed );
                            toReturn.push_back( onLine );
                        }
                    }
                }

                foundPointOnRightSide = true;
                toReturn.push_back( p );
            }
        }
    }

    if( alignAtEndOrStart ) {
        std::reverse( toReturn.begin(), toReturn.end() );
    }
    return toReturn;
}

/// Fit a degree-3 curve fit to 'samples', or fall back on other approaches if 'samples' is not large enough. If
/// it's not possible to create a curve with nonzero length, return nullptr.
std::unique_ptr< Curve > fitCurveToData( const Vec2Vec& samples )
{
    const auto numSamples = samples.size();
    switch( numSamples ) {
    case 0:
    case 1: {
        return nullptr;
    }
    case 2: {
        return Curve::lineSeg( samples[ 0 ], samples[ 1 ] );
    }
    case 3: {
        return Curve::spline( 2, samples );
    }
    case 4: {
        return Curve::spline( 3, samples );
    }
    case 5: {
        return Curve::spline( 3, samples );
    }
    case 6:
    case 7: {
        return Curve::createFitToDataPoints( 2, 3, samples, core::CurveFitParametrizeType::SplitIntervalEvenly );
    }
    default: {
        const size_t samplesPerControlPoint = 5;

        const size_t numForOneCurve = 4;
        const size_t numIdeal = static_cast< int >( numSamples / samplesPerControlPoint );
        const size_t numMaxAllowed = numSamples - 4;

        const auto numControl = std::min< size_t >(
                    numMaxAllowed,
                    std::max< size_t >( numForOneCurve, numIdeal ) );

        return Curve::createFitToDataPoints(
            3,
            static_cast< int >( numControl ),
            samples,
            core::CurveFitParametrizeType::SplitIntervalEvenly );
    }
    }
}

} // unnamed
	
CurvesPostScript::CurvesPostScript( const core::BoundingBoxd& canvas, boost::optional< double > psMinDimOpt ) :
    _canvasBounds( canvas ),
    _currentRGB{ 0, 0, 0 },
    _currentLineWidthPS( -1.0 )
{
    const double psMinDim = psMinDimOpt.is_initialized() ? psMinDimOpt.value() : std::min( canvas.widthExclusive(), canvas.heightExclusive() );

    const double psWidth = canvas.widthExclusive() < canvas.heightExclusive()
        ? psMinDim
        : psMinDim * canvas.widthExclusive() / canvas.heightExclusive();
    const double psHeight = canvas.heightExclusive() < canvas.widthExclusive()
        ? psMinDim
        : psMinDim * canvas.heightExclusive() / canvas.widthExclusive();

    _psBounds = core::BoundingBoxd( core::Vector2( 0, 0 ), core::Vector2( psWidth, psHeight ) );

    _stream << "%!PS-Adobe-2.0 EPSF-1.2\n";
    setDefaultPrecision( _stream );

    _stream << "<< /PageSize ["
        << _psBounds.widthExclusive()
        << " "
        << _psBounds.heightExclusive()
        << "] >> setpagedevice\n";

    setLineWidth( 1.0 );
    setColor( 0, 0, 0 );
}

const core::BoundingBoxd& CurvesPostScript::canvasBounds() const
{
    return _canvasBounds;
}

void CurvesPostScript::addCircle( const core::Vector2& canvasSpace, double radiusCanvas, bool strokeOrFill )
{
    const core::Vector2 ps = canvasToPS( canvasSpace );

    // We're assuming no aspect ratio difference between the PostScript doc and
    // the region of canvas depicted.
    const double radiusPS = canvasToPS( radiusCanvas );

    core::BoundingBoxd itemBounds(
        ps - core::Vector2( radiusCanvas, radiusCanvas ),
        ps + core::Vector2( radiusCanvas, radiusCanvas ) );
    itemBounds.expand( _currentLineWidthPS );

    if( _psBounds.intersects( itemBounds ) ) {
        _stream << ps.x()
            << " "
            << ps.y()
            << " "
            << radiusPS
            << " newpath 0 360 arc closepath "
            << ( strokeOrFill ? "stroke" : "fill" )
            << "\n";
    }
}

void CurvesPostScript::addCurve( ConstCurve canvasSpace )
{
    const OutputCurve inPS = canvasToPS( canvasSpace );
    auto itemBounds = inPS->boundingBox();
    itemBounds.expand( _currentLineWidthPS );
    if( _psBounds.intersects( itemBounds ) ) {
        const std::string curveCode = eps( *inPS, true, true );
        _stream << "newpath\n" << curveCode << "\n";
    }
}

void CurvesPostScript::addLineSegment( const core::Vector2& a, const core::Vector2& b )
{
    const auto aPS = canvasToPS( a );
    const auto bPS = canvasToPS( b );
    core::BoundingBoxd itemBounds( aPS, bPS );
    itemBounds.expand( _currentLineWidthPS );
    if( _psBounds.intersects( itemBounds ) ) {
        _stream << "\nnewpath ";
        printPoint( aPS, _stream );
        _stream << " moveto ";
        printPoint( bPS, _stream );
        _stream << " lineto closepath stroke\n";
    }
}

void CurvesPostScript::strokeAndOrFillPath( boost::optional< RGB > strokeColor, boost::optional< RGB > fillColor )
{
    const auto oldRGB = _currentRGB;
    if( strokeColor.is_initialized() ) {
        if( fillColor.is_initialized() ) {
            _stream << "gsave\n";
        }
        setColor( strokeColor.value() );
        _stream << " stroke ";
        if( fillColor.is_initialized() ) {
            _stream << "\ngrestore ";
        }
    }
    if( fillColor.is_initialized() ) {
        setColor( fillColor.value() );
        _stream << "eofill ";
    }
    _stream << "\n";
    setColor( oldRGB );
}

void CurvesPostScript::setColor( const RGB& rgb )
{
    setColor( rgb[ 0 ], rgb[ 1 ], rgb[ 2 ] );
}

void CurvesPostScript::setColor( const core::Vector3 &rgb )
{
    setColor( core::mathUtility::rgbFloatToInt( rgb ) );
}

void CurvesPostScript::setColor( int r, int g, int b )
{
    if( r != _currentRGB[ 0 ] || g != _currentRGB[ 1 ] || b != _currentRGB[ 2 ] ) {

        auto toF = [ & ]( int channel )
        {
            return static_cast< double > ( channel ) / 255.0;
        };

        setColorPrecision( _stream );
        _stream << toF( r ) << " " << toF( g ) << " " << toF( b ) << " setrgbcolor\n";
        setDefaultPrecision( _stream );

        _currentRGB[ 0 ] = r;
        _currentRGB[ 1 ] = g;
        _currentRGB[ 1 ] = b;
    }
}

void CurvesPostScript::setLineWidth( double canvasUnits )
{
    const double newWidthPS = canvasToPS( canvasUnits );
    if( !core::mathUtility::closeEnough( _currentLineWidthPS, newWidthPS ) ) {
        _stream << newWidthPS << " setlinewidth\n";
        _currentLineWidthPS = newWidthPS;
    }
}

double CurvesPostScript::lineWidth() const
{
    return psToCanvas( _currentLineWidthPS );
}

std::string CurvesPostScript::epsCode() const
{
    return _stream.str();
}

double CurvesPostScript::canvasToPS( double canvas ) const
{
    return canvas * _psBounds.widthExclusive() / _canvasBounds.widthExclusive();
}

double CurvesPostScript::psToCanvas( double ps ) const
{
    return ps * _canvasBounds.widthExclusive() / _psBounds.widthExclusive();
}

core::Vector2 CurvesPostScript::canvasDirToPS( const core::Vector2& v )
{
    return core::Vector2{ v.x(), -v.y() };
}

core::Vector2 CurvesPostScript::canvasToPS( const core::Vector2& canvasPos ) const
{
    core::Vector2 ps(
        _psBounds.xMin() + _psBounds.widthExclusive() * ( canvasPos.x() - _canvasBounds.xMin() ) / _canvasBounds.widthExclusive(),
        _psBounds.yMin() + _psBounds.heightExclusive() * ( canvasPos.y() - _canvasBounds.yMin() ) / _canvasBounds.heightExclusive() );

    // Postscript has (0,0) as bottomleft, not topleft.
    ps.setY( _psBounds.heightExclusive() - ps.y() );
    return ps;
}

CurvesPostScript::OutputCurve CurvesPostScript::canvasToPS( ConstCurve canvas ) const
{
    OutputCurve ps = std::make_unique< OutputCurve::element_type >( canvas );
    const auto canvasToPSWrapper = [ & ]( const core::Vector2& canvas )
    {
        return canvasToPS( canvas );
    };
    ps->transform( canvasToPSWrapper );
    return ps;
}

void CurvesPostScript::addVaryingWidthCurve(
    ConstCurve canvasSpace, CanvasPosToWidth posToWidth, SamplesPerInterval samplesPerInterval, const StrokeProperties& props )
{
    addVaryingWidthCurve( canvasSpace,
        [ &posToWidth ]( double, const core::Vector2& canvasPos )
        {
            return posToWidth( canvasPos );
        },
        samplesPerInterval,
        props );
}

void CurvesPostScript::addVaryingWidthCurve(
    ConstCurve canvasSpace, ParamToWidth paramToWidth, SamplesPerInterval samplesPerInterval, const StrokeProperties& props )
{
    addVaryingWidthCurve( canvasSpace,
        [ &paramToWidth ]( double t, const core::Vector2& )
        {
            return paramToWidth( t );
        },
        samplesPerInterval,
        props );
}

void CurvesPostScript::addVaryingWidthCurve(
    ConstCurve canvasSpace, LowLevelWidthFunctor widthFunctor, SamplesPerInterval samplesPerInterval, const StrokeProperties& props )
{
    // Fit a spline to the left side and a spline to the right side using sampled widths. But we can't just
    // pick one simple number of samples since 'canvasSpace' may have some C0 "seams" and we should handle each
    // subcurve independently, unless this is specifically excluded via props.treatAsContinuous.

    OutputCurve inPS = canvasToPS( canvasSpace );

    LowLevelWidthFunctor psWidth = [ & ]( double t, const Vec2& canvas )
    {
        return canvasToPS( widthFunctor( t, canvas ) );
    };

    std::vector< std::vector< core::Vector2 > > leftPolylines, rightPolylines;
    size_t numSubcurves = 1;
    if( props.treatAsContinuous ) {
        // Get T-values
        const auto numSamples = samplesPerInterval( core::BoundingIntervald{ 0.0, 1.0 } );
        std::vector< double > t( numSamples );
        std::vector< core::Vector2 > pos( numSamples );
        std::vector< double > width( numSamples );
        for( size_t i = 0; i < numSamples; i++ ) {
            const auto tVal = F_FROM_I( i, numSamples );
            t[ i ] = tVal;
            pos[ i ] = inPS->position( tVal );
            width[ i ] = psWidth( tVal, pos[ i ] );
        }

        leftPolylines.resize( 1 );
        rightPolylines.resize( 1 );
        miteredOffsetSamples( pos, width, leftPolylines.front(), rightPolylines.front() );
    } else {
        offsetSamplesForMiteredJoinRender( *inPS, leftPolylines, rightPolylines, samplesPerInterval, psWidth );
        numSubcurves = leftPolylines.size();
    }

    // For against-wall alignment (which may not be applicable).
    const auto startShiftDir = canvasDirToPS( canvasSpace.derivative( 0.0 ) );
    const auto endShiftDir = canvasDirToPS( canvasSpace.derivative( 1.0 ) );

    const auto solveForSide = [ & ]( bool leftOrRight ) -> OutputCurve
    {
        // On one side or the other of 'inPS', solve for 'numSubcurves' offset curves (or fewer) and then stitch them together.
        std::vector< OutputCurve > subcurves;
        for( size_t i = 0; i < numSubcurves; i++ ) {
            auto offsetSamples = leftOrRight ? leftPolylines[ i ] : rightPolylines[ i ];

            // Optionally alter the first and/or last subcurves' samples so that the resulting stroke appears to run up cleanly
            // against "walls" indicated at each end.
            {
                const auto originalOffsetSamples = offsetSamples;
                // Do this for the first subcurve.
                if( i == 0 && props.startCap.wallNormal.is_initialized() ) {
                    const auto onWall = inPS->startPosition();
                    const auto startWidth = psWidth( 0.0, onWall );
                    offsetSamples = alignSamplesAgainstWall(
                        offsetSamples,
                        onWall,
                        canvasDirToPS( props.startCap.wallNormal.value() ),
                        false,
                        startShiftDir,
                        startWidth );
                }

                // Do this for the last subcurve.
                if( i == numSubcurves - 1 && props.endCap.wallNormal.is_initialized() ) {
                    const auto onWall = inPS->endPosition();
                    const auto endWidth = psWidth( 1.0, onWall );
                    offsetSamples = alignSamplesAgainstWall(
                        offsetSamples,
                        onWall,
                        canvasDirToPS( props.endCap.wallNormal.value() ),
                        true,
                        endShiftDir,
                        endWidth );
                }

                // If 'alignSamplesAgainstWall' shaved off too much of 'offsetSamples', just reverse that.
                if( offsetSamples.size() < originalOffsetSamples.size() / 2 ) {
                    offsetSamples = originalOffsetSamples;
                }
            }

            auto subcurve = fitCurveToData( offsetSamples );
            if( subcurve ) {
                subcurves.push_back( std::move( subcurve ) );
            }
        }

        // Stitch them together. The precision doesn't really matter because the parametrization doesn't matter, only the curve as a whole.
        // Only curve geometry matters in this context (though knot multiplicity does obviously mean an increase in PDF size).
        if( subcurves.size() ) {
            OutputCurve wholeSide = core::BSpline2Utility::stitchC0Spline( 
				core::uniquesToConstRaws( subcurves ), core::BSpline2::defaultLengthPrecision );
            return wholeSide;
        } else {
            return nullptr;
        }
    };

    OutputCurve left = solveForSide( true );
    OutputCurve right = solveForSide( false );

    // There's a possibility that against-wall alignment completely erases one side or both, so make
    // sure both side curves exist.
    if( left && right ) {
        right->reverse();

        const bool inBounds = left->boundingBox().intersects( _psBounds )
            || right->boundingBox().intersects( _psBounds );

        if( inBounds ) {
            _stream << "newpath\n";
            _stream << eps( *left, true, false ) << "\n";
            _stream << eps( *core::BSpline2::polyline( { left->endPosition(), right->startPosition() } ), false, false )
                    << "\n";
            _stream << eps( *right, false, false ) << "\n";
            _stream << eps( *core::BSpline2::polyline( { right->endPosition(), left->startPosition() } ), false, false )
                    << "\n";
            _stream << "fill\n";
        }
    }
}

/// Generate the EPS for a single curve.
std::string CurvesPostScript::eps( ConstCurve c, bool initialMoveTo, bool strokeAtEnd )
{
    const int psDegree = 3; // Bezier curves must be cubic in PostScript.
    if( c.degree() > psDegree ) {
        return "";
    }

    OutputCurve cMod = std::make_unique< OutputCurve::element_type >( c );
    if( cMod->degree() < psDegree ) {
        cMod->degreeElevate( psDegree );
    }

    std::vector< double > unused;
    const std::vector< std::vector< core::Vector2 > > bezierControl = cMod->breakIntoBCurves( unused );

    if( bezierControl.size() == 0 ) {
        return "";
    }

    std::stringstream str;
    setDefaultPrecision( str );
    str << " ";

    if( initialMoveTo ) {
        printPoint( bezierControl[ 0 ][ 0 ], str );
        str << " moveto ";
    }

    for( const auto& bezier : bezierControl ) {
        for( size_t i = 1; i < bezier.size(); i++ ) {
            printPoint( bezier[ i ], str );
            str << " ";
        }
        str << " curveto ";
    }

    if( strokeAtEnd ) {
        str << " stroke";
    }
    return str.str();
}

} // printCurves
