#include <utility/bspline2.h>

#include <utility/boundingbox.h>
#include <utility/bspline2utility.h>
#include <utility/buildsplineexception.h>
#include <utility/casts.h>
#include <utility/curveinterval.h>
#include <utility/linesegment.h>
#include <utility/mathutility.h>

#include <GTE/Mathematics/BSplineCurve.h>
#include <GTE/Mathematics/BSplineCurveFit.h>

#include <Eigen/Sparse>

#include <boost/numeric/conversion/cast.hpp>

#include <algorithm>

namespace core {

namespace {

using GteVec2 = gte::Vector< 2, double >;
using GteControlPoints = std::vector< GteVec2 >;

GteVec2 gteVec2( const Vector2& p )
{
    return GteVec2{ p.x(), p.y() };
}

/// Get a vector of control points in GTE form
GteControlPoints gtePoints( const std::vector< Vector2 >& toCopy )
{
    GteControlPoints toReturn( toCopy.size() );
    for( size_t i = 0; i < toCopy.size(); i++ ) {
        toReturn[ i ] = gteVec2( toCopy[ i ] );
    }
    return toReturn;
}

/// Duplicate the knot at 'i' in 'knots'. If there are multiple copies of this knot,
/// then 'i' must index the first of them.
void doubleKnot(
    int i, int numExistingCopies, int degree, std::vector< double >& knots, BSpline2::Control& control )
{
    const double knotToDouble = knots[ i ];
    BSpline2Utility::insertKnot( i, knotToDouble, numExistingCopies, degree, knots, control );
}

/// Return whether a spline can be built with 'degree' and 'control'.
bool validateControlPoints( int degree, const BSpline2::Control& control )
{
    return degree > 0 && control.size() >= degree + 1;
}

} // unnamed

BSpline2::BSpline2()
    : _degree( 0 )
    , _cachedPrecision( 0 )
    , _cachedLength( 0 )
{
}

BSpline2::BSpline2( int degree, const Control& controlPoints ) : BSpline2()
{    
    buildFromControlPoints( degree, controlPoints );
}

BSpline2::BSpline2( int degree, const Control& controlPoints, const std::vector< double >& intermediateKnots )
    : BSpline2()
{
    buildFromControlPointsAndKnots( degree, controlPoints, intermediateKnots );
}

BSpline2::BSpline2( const Type& other)
{
    *this = other;
}

BSpline2::BSpline2( Type&& other)
{
    *this = other;
}

BSpline2::~BSpline2()
{
}

BSpline2& BSpline2::operator = ( const Type& other )
{
    // This relies on my edit to GTE to allow copying gte::BasisFunction.
    _gteSpline = std::make_unique< GTESpline >( *other._gteSpline );
    _degree = other._degree;
    _controlPoints = other._controlPoints;
    _cachedLength = other._cachedLength;
    _cachedPrecision = other._cachedPrecision;
    return *this;
}

BSpline2& BSpline2::operator = ( Type&& other )
{
    _gteSpline = std::move( other._gteSpline );
    _degree = other._degree;
    _controlPoints = std::move( other._controlPoints );
    _cachedLength = other._cachedLength;
    _cachedPrecision = other._cachedPrecision;
    return *this;
}

void BSpline2::buildFromControlPoints( int degree, const Control& controlPoints )
{
    _cachedLength = 0;
    _cachedPrecision = 0;

    if( validateControlPoints( degree, controlPoints ) ) {
        // This constructor makes uniformly spaced knots.
        gte::BasisFunctionInput< double > bfiUniform(
                    static_cast< int32_t >( controlPoints.size() ),
                    static_cast< int32_t >( degree ) );
        const auto gteControlPoints = gtePoints( controlPoints );
        _gteSpline = std::make_unique< GTESpline >( bfiUniform, &gteControlPoints[ 0 ] );
        _degree = degree;
        _controlPoints = controlPoints;
    } else {
        throw BuildSplineException( "Degree and control points do not match" );
    }
}

void BSpline2::buildFromControlPointsAndKnots(
    int degree, const Control& controlPoints, const std::vector< double >& intermediateKnots )
{
    _cachedLength = 0;
    _cachedPrecision = 0;

    if( validateControlPoints( degree, controlPoints )
        && intermediateKnots.size() == controlPoints.size() - degree - 1 ) {

        // Get rid of any multiple knots at beginning or end of the spline. GTE is not built
        // to accommodate these--see line 195 in BasisFunction.h, where if a degree-3 spline's first
        // knot has multiplicity greater than 4, there is an error. In fact, the multiplicity of
        // that knot _has_ to be four, otherwise the spline will be interpreted as closed, which
        // is not something my code is meant to deal with.
        //
        // In short, we need to be extra strict about the T=0 and T=1 knots.

        Control filteredControl = controlPoints;
        std::vector< double > filteredKnots = intermediateKnots;
        while( filteredKnots.size() && mathUtility::closeEnough( filteredKnots.front(), 0.0 ) ) {
            filteredKnots.erase( filteredKnots.begin() );
            filteredControl.erase( filteredControl.begin() );
        }
        while( filteredKnots.size() && mathUtility::closeEnough( filteredKnots.back(), 1.0 ) ) {
            filteredKnots.erase( filteredKnots.end() - 1  );
            filteredControl.erase( filteredControl.end() - 1 );
        }

        if( filteredKnots.size() ) {
            // Do the more involved GTE curve creation that involves specifying all the knots.
            gte::BasisFunctionInput< double > bfi;
            bfi.degree = degree;
            bfi.uniform = false;
            bfi.periodic = false;
            bfi.uniqueKnots.resize( 0 );

            // First put in a knot for T=0. Must have multiplicity degree+1 for
            // the curve to be interpreted as open by GTE.
            {
                gte::UniqueKnot< double > firstKnot;
                firstKnot.multiplicity = static_cast< int32_t >( degree + 1 );
                firstKnot.t = 0.0;
                bfi.uniqueKnots.push_back( firstKnot );
            }

            // Clean up internal knots so none have multiplicity > degree+1.
            {
                // This is something we have to enforce because of GTE. WildMagic didn't care.
                const int32_t maxMultiplicity = degree + 1;

                size_t fnIdx = 0; // 'filteredKnots' index.
                size_t controlIdxToDelete = degree; // Index in 'controlPoints' for deleting redundant control points.
                while( true ) {

                    const auto knotToAdd = filteredKnots[ fnIdx ];
                    auto idxOfLastMultiple = fnIdx;
                    while( true ) {
                        if( idxOfLastMultiple == filteredKnots.size() - 1 ) {
                            break;
                        }
                        if( !mathUtility::closeEnough( knotToAdd, filteredKnots[ idxOfLastMultiple + 1 ] ) ) {
                            break;
                        }
                        idxOfLastMultiple++;
                    }

                    const auto multiplicity = idxOfLastMultiple - fnIdx + 1;
                    const auto correctedMultiplicity = std::min< int32_t >(
                        static_cast< int32_t >( multiplicity ),
                        maxMultiplicity );

                    // Delete extra knots by deleting control points.
                    for( auto i = correctedMultiplicity; i < multiplicity; i++ ) {
                        filteredControl.erase( filteredControl.begin() + controlIdxToDelete );
                    }
                    controlIdxToDelete += correctedMultiplicity;

                    // DO SOMETHING BRANCH-DEPENDENT
                    gte::UniqueKnot< double > knot;
                    knot.multiplicity = correctedMultiplicity;
                    knot.t = knotToAdd;
                    bfi.uniqueKnots.push_back( knot );

                    // Are we done?
                    if( idxOfLastMultiple == filteredKnots.size() - 1 ) {
                        break;
                    }
                    fnIdx = idxOfLastMultiple + 1;
                }
            }
            bfi.numControls = static_cast< int32_t >( filteredControl.size() );

            // Put in the last knot for T = 1.
            {
                gte::UniqueKnot< double > lastKnot;
                lastKnot.multiplicity = static_cast< int32_t >( degree + 1 );
                lastKnot.t = 1.0;
                bfi.uniqueKnots.push_back( lastKnot );
            }
            bfi.numUniqueKnots = static_cast< int32_t >( bfi.uniqueKnots.size() );

            const auto gteControlPoints = gtePoints( filteredControl );
            _gteSpline = std::make_unique< GTESpline >( bfi, &gteControlPoints[ 0 ] );
            _degree = degree;
            _controlPoints = filteredControl;
        } else {
            buildFromControlPoints( degree, filteredControl );
        }
    } else {
        throw BuildSplineException( "Parameters invalid" );
    }
}

BSpline2::UniquePtr BSpline2::spline( int degree, const Control& control, const std::vector< double >& intermediateKnots )
{
    return createFromControlPointsAndKnots( degree, control, intermediateKnots );
}

BSpline2::UniquePtr BSpline2::spline( int degree, const Control& control )
{
    return std::make_unique< Type >( degree, control );
}

BSpline2::UniquePtr BSpline2::polyline( const Control& control )
{
    return createFromControlPoints( 1, control );
}

BSpline2::UniquePtr BSpline2::boxPolyline( const BoundingBoxd& b )
{
    return polyline(
    {
        b.topLeft(),
        b.topRight(),
        b.bottomRight(),
        b.bottomLeft(),
        b.topLeft()
    } );
}

BSpline2::UniquePtr BSpline2::lineSeg( const Vector2& a, const Vector2& b )
{
    return polyline( { a, b } );
}

BSpline2::UniquePtr BSpline2::lineSeg( const LineSegment& seg )
{
    return lineSeg( seg.a, seg.b );
}

const BSpline2::Control& BSpline2::controlPoints() const
{
    return _controlPoints;
}

int BSpline2::degree() const
{
    return _degree;
}

size_t BSpline2::numInternalKnots( int degree, size_t numControl )
{
    if( numControl < degree + 1 ) {
        return 0;
    } else {
        return numControl - degree - 1;
    }
}

BSpline2::UniquePtr BSpline2::forceClosed() const
{
    auto control = controlPoints();
    if( control.size() > 1 ) {
        control.back() = control.front();
        return spline( degree(), control, internalKnots() );
    } else {
        return clone();
    }
}

std::vector< double > BSpline2::fullKnotsNoMultiples() const
{
    auto internal = internalKnots();
    std::set< double > asSet( internal.begin(), internal.end() );
    asSet.emplace( 0.0 );
    asSet.emplace( 1.0 );
    return std::vector< double >( asSet.begin(), asSet.end() );
}

std::vector< double > BSpline2::tForPolylineApproxLimited( size_t defaultNumPoints, size_t maxPoints ) const
{
    const auto knots = fullKnotsNoMultiples();

    std::vector< double > toReturn;
    if( knots.size() > maxPoints ) {
        // Include 0.0 no matter what.
        toReturn.push_back( 0.0 );
        if( knots.size() > 2 ) {
            // Out of the knots between 0 and 1, pick the ones with the largest adjacent T-intervals.
            using WeightAndKnot = std::pair< double, double >;
            std::set< WeightAndKnot > internalKnotsWithWeights;
            for( size_t i = 1; i < knots.size() - 1; i++ ) {
                const auto weight = ( knots[ i ] - knots[ i - 1 ] ) + ( knots[ i + 1 ] - knots[ i ] );
                internalKnotsWithWeights.emplace( WeightAndKnot{ weight, knots[ i ] } );
            }

            const size_t numInternalKnotsAllowed = maxPoints - 2;
            std::set< double > selectedInternal;
            for( auto rit = internalKnotsWithWeights.rbegin(); rit != internalKnotsWithWeights.rend(); rit++ ) {
                if( selectedInternal.size() == numInternalKnotsAllowed ) {
                    break;
                }
                selectedInternal.emplace( rit->second );
            }
            for( const auto& internal : selectedInternal ) {
                toReturn.push_back( internal );
            }
        }
        // Include 1.0 no matter what.
        toReturn.push_back( 1.0 );
    } else {
        const auto pointsLeft = maxPoints - knots.size();
        toReturn.push_back( 0.0 );
        for( size_t i = 1; i < knots.size(); i++ ) {

            // Weight of this interval relative to others.
            const double intervalWeight = ( knots[ i ] - knots[ i - 1 ] );

            const size_t numInsideIntervalPoints = std::min< size_t >(
                static_cast< size_t >( static_cast< double >( defaultNumPoints ) * intervalWeight ),
                static_cast< size_t >( static_cast< double >( pointsLeft ) * intervalWeight ) );
            for( size_t j = 0; j < numInsideIntervalPoints; j++ ) {
                const double f = static_cast< double >( j + 1 ) / static_cast< double >( numInsideIntervalPoints + 1 );
                toReturn.push_back( mathUtility::lerp( knots[ i - 1 ], knots[ i ], f ) );
            }

            toReturn.push_back( knots[ i ] );
        }
    }

    return toReturn;
}

std::vector< double > BSpline2::tForPolylineApprox( const std::array< double, 2 >& interval, size_t defaultNumPoints ) const
{
    // Pretend everything is increasing now.
    const bool increasing = interval[ 1 ] >= interval[ 0 ];
    // Used to be knots = internalKnots(), but that involves wasting points on the boundaries of 0-length intervals.
    const auto knots = fullKnotsNoMultiples();

    const double tMin = increasing ? interval[ 0 ] : interval[ 1 ];
    const double tMax = increasing ? interval[ 1 ] : interval[ 0 ];

    std::vector< double > criticalT{ tMin };
    for( const auto knot : knots ) {
        if( knot > tMin && knot < tMax ) {
            if( !mathUtility::closeEnough( knot, criticalT.back() ) ) {
                criticalT.push_back( knot );
            }
        }
    }
    if( mathUtility::closeEnough( criticalT.back(), tMax ) ) {
        criticalT[ criticalT.size() - 1 ] = tMax;
    } else {
        criticalT.push_back( tMax );
    }

    std::vector< double > toReturn;
    for( size_t i = 1; i < criticalT.size(); i++ ) {
        const bool lastInterval = ( i == criticalT.size() - 1 );

        const size_t numPointsTentative = static_cast< size_t >( static_cast< double >( defaultNumPoints )
            * ( criticalT[ i ] - criticalT[ i - 1 ] )
            / ( tMax - tMin ) );
        const size_t numPoints = std::max< size_t >( 2, numPointsTentative );
        for( size_t j = 0; j < ( lastInterval ? numPoints : numPoints - 1 ); j++ ) {
            const double f = static_cast< double >( j ) / static_cast< double >( numPoints - 1 );
            toReturn.push_back( mathUtility::lerp( criticalT[ i - 1 ], criticalT[ i ], f ) );
        }
    }

    if( !increasing ) {
        std::reverse( toReturn.begin(), toReturn.end() );
    }
    return toReturn;
}

std::vector< Vector2 > BSpline2::crudePolylineApproximation( size_t numPoints ) const
{
    std::vector< Vector2 > toReturn( numPoints );
    for( size_t i = 0; i < numPoints; i++ ) {
        const auto t = F_FROM_I( i, numPoints );
        toReturn[ i ] = position( t );
    }
    return toReturn;
}

std::vector< Vector2 > BSpline2::polylineApproximation( int numPoints, bool ignoreNumPointsIfDeg1 ) const
{
    if( _degree == 1 && ignoreNumPointsIfDeg1 ) {
        return _controlPoints;
    } else {
        const auto tValues = tForPolylineApprox( { 0.0, 1.0 }, numPoints );
        std::vector< Vector2 > toReturn( tValues.size() );
        for( size_t i = 0; i < tValues.size(); i++ ) {
            toReturn[ i ] = position( tValues[ i ] );
        }
        return toReturn;
    }
}

double BSpline2::cachedLength( size_t precision ) const
{
    if( precision > 1 ) {
        if( precision > _cachedPrecision ) {

            // If the spline is just a line segment, just return the exact length.
            if( _degree == 1 && _controlPoints.size() == 2 ) {
                _cachedLength = ( _controlPoints[ 1 ] - _controlPoints[ 0 ] ).length();
                _cachedPrecision = std::numeric_limits< size_t >::max();
            } else {
                _cachedLength = 0;
                Vector2 currentPos = position( 0 );
                for( size_t i = 1; i < precision; i++ ) {
                    double t = boost::numeric_cast< double >( i ) / boost::numeric_cast< double >( precision - 1 );
                    const Vector2 nextPos = position( t );
                    _cachedLength += ( nextPos - currentPos ).length();
                    currentPos = nextPos;
                }
                _cachedPrecision = precision;
            }
        }
        return _cachedLength;
    } else {
        return 0;
    }
}

const Vector2& BSpline2::startPosition() const
{
    return _controlPoints.front();
}

const Vector2& BSpline2::endPosition() const
{
    return _controlPoints.back();
}

const Vector2& BSpline2::endpoint( bool startOrEnd ) const
{
    return startOrEnd ? _controlPoints.front() : _controlPoints.back();
}

bool BSpline2::endpointsEqual() const
{
    return _controlPoints.front() == _controlPoints.back();
}

Vector2 BSpline2::position( double t ) const
{
    if( t == 0.0 ) {
        return _controlPoints.front();
    } else if( t == 1.0 ) {
        return _controlPoints.back();
    } else {
        GteControlPoints jet( 1 );
        _gteSpline->Evaluate( t, 0, &jet[ 0 ] );
        return Vector2( jet.front()[ 0 ], jet.front()[ 1 ] );
    }
}

Vector2 BSpline2::derivative(double t) const
{
    GteControlPoints jet( 2 );
    _gteSpline->Evaluate( t, 1, &jet[ 0 ] );
    return Vector2( jet.back()[ 0 ], jet.back()[ 1 ] );
}

Vector2 BSpline2::secondDerivative(double t) const
{
    GteControlPoints jet( 3 );
    _gteSpline->Evaluate( t, 2, &jet[ 0 ] );
    return Vector2( jet.back()[ 0 ], jet.back()[ 1 ] );
}

double BSpline2::curvatureSigned( double t ) const
{
    // From "High accuracy geometric Hermite interpolation."
    const Vector2 first = derivative( t );
    const double firstMag = first.length();
    const Vector2 second = secondDerivative( t );
    return Vector2::crossProductZ( first, second ) / pow( firstMag, 3.0 );
}

double BSpline2::curvatureMagnitude( bool startOrEnd ) const
{
    if( degree() < 2 ) {
        return 0.0;
    }
    std::vector< double > unused;
    const std::vector< Control > bezierControls = breakIntoBCurves( unused );

    // Using Sederberg's definition: pg. 31 of his book.
    double h = 0.0;
    double a = 0.0;
    if( startOrEnd ) {
        const Control& firstBezier = bezierControls.front();
        const Vector2 aToB = firstBezier[ 1 ] - firstBezier[ 0 ];
        a = aToB.length();
        const Vector2 projOntoLine =
            mathUtility::projectPointOntoLine( firstBezier[ 2 ], firstBezier[ 0 ], firstBezier[ 1 ] );
        h = ( firstBezier[ 2 ] - projOntoLine ).length();
    } else {
        const Control& lastBezier = bezierControls.back();
        const Vector2 aToB = lastBezier[ lastBezier.size() - 2 ] - lastBezier[ lastBezier.size() - 1 ];
        a = aToB.length();
        const Vector2 projOntoLine =
            mathUtility::projectPointOntoLine( lastBezier[ lastBezier.size() - 3 ], lastBezier[ lastBezier.size() - 1 ], lastBezier[ lastBezier.size() - 2 ] );
        h = ( lastBezier[ lastBezier.size() - 3 ] - projOntoLine ).length();
    }
    if( mathUtility::closeEnoughToZero( a ) ) {
        return std::numeric_limits< double >::infinity();
    } else {
        return static_cast< double >( degree() - 1 )
            / static_cast< double >( degree() )
            * h
            / ( a * a );
    }
}

void BSpline2::degreeElevate( int degree )
{
    if( _degree < degree ) {

        std::vector< double > unused;
        const std::vector< Control > bezierCurves = breakIntoBCurves( unused );

        // Degree-elevate an individual Bezier curve (from Sederberg's book).
        auto elevateBezier = [ & ]( const Control& original ) -> Control
        {
            Control toElevate = original;
            for( int elevate = 1; elevate <= degree - _degree; elevate++ ) {
                Control elevated( toElevate.size() + 1 );
                elevated.front() = toElevate.front();
                elevated.back() = toElevate.back();

                for( int i = 1; i <= toElevate.size() - 1; i++ ) {
                    const double alpha = static_cast< double >( i ) / static_cast< double >( _degree + elevate );
                    elevated[ i ] = toElevate[ i - 1 ] * alpha + toElevate[ i ] * ( 1.0 - alpha );
                }
                toElevate = std::move( elevated );
            }
            return toElevate;
        };

        const int numCurves = static_cast< int >( bezierCurves.size() );
        const int pointsPerCurve = degree + 1;
        Control newControl( numCurves * ( pointsPerCurve - 1 ) + 1 );
        for( int c = 0; c < numCurves; c++ ) {
            const Control elevatedBezierControl = elevateBezier( bezierCurves[ c ] );
            std::copy( elevatedBezierControl.begin(),
                       elevatedBezierControl.end(),
                       c == 0 ? newControl.begin() : newControl.begin() + c * ( pointsPerCurve - 1 ) );
        }

        // Get the original knot vector with all the duplicates removed (since 'bezierCurves' doesn't contain any of
        // this spline's degenerate curves).
        std::vector< double > interKnots = internalKnots();
        int i = 0;
        while( i < static_cast< int >( interKnots.size() ) - 1 ) {
            if( mathUtility::closeEnough( interKnots[ i ], interKnots[ i + 1 ] ) ) {
                interKnots.erase( interKnots.begin() + i );
            } else {
                i++;
            }
        }

        // Just tie the curves back together in a C0 fashion.
        std::vector< double > duplicatedKnots( interKnots.size() * degree );
        for( int i = 0; i < static_cast< int >( interKnots.size() ); i++ ) {
            for( int j = 0; j < degree; j++ ) {
                duplicatedKnots[ i * degree + j ] = interKnots[ i ];
            }
        }
        buildFromControlPointsAndKnots( degree, newControl, duplicatedKnots );
    }
}

void BSpline2::transform( std::function< Vector2( const Vector2& ) > f )
{
    for( auto& control : _controlPoints ) {
        control = f( control );
    }
    buildFromControlPointsAndKnots( _degree, _controlPoints, internalKnots() );
}

void BSpline2::scale( const Vector2& scaleBy )
{
    for( auto& control : _controlPoints ) {
        control.scale( scaleBy );
    }
    buildFromControlPointsAndKnots( _degree, _controlPoints, internalKnots() );
}

void BSpline2::reverse()
{
    std::reverse( _controlPoints.begin(), _controlPoints.end() );
    auto knots = internalKnots();
    std::reverse( knots.begin(), knots.end() );
    for( double& knot : knots ) {
        knot = 1.0 - knot;
    }
    buildFromControlPointsAndKnots( _degree, _controlPoints, knots );
}

BoundingBoxd BSpline2::boundingBox() const
{
    return BoundingBoxd( _controlPoints );
}

std::vector< double > BSpline2::internalKnots() const
{
    const auto& bfi = _gteSpline->GetBasisFunction();
    const auto numUniqueKnots = bfi.GetNumUniqueKnots();
    if( numUniqueKnots == 0 ) {
        return {};
    }

    const auto* const uniqueKnots = bfi.GetUniqueKnots();

    std::vector< double > toReturn;

    // Add multiples of T=0.
    {
        // First knot has multiplicity degree+1 by default
        const auto& knot = *uniqueKnots;
        for( int i = _degree + 1; i < knot.multiplicity; i++ ) {
            toReturn.push_back( knot.t );
        }
    }

    // Add all the in-between knots.
    {
        // unique-knot index
        for( int ukIdx = 1; ukIdx < numUniqueKnots - 1; ukIdx++ ) {
            const auto& knot = *( uniqueKnots + ukIdx );
            for( int i = 0; i < knot.multiplicity; i++ ) {
                toReturn.push_back( knot.t );
            }
        }
    }

    // Add multiples of T=1.
    {
        // Same idea as with first knot.
        const auto& knot = *( uniqueKnots + numUniqueKnots - 1 );
        for( int i = _degree + 1; i < knot.multiplicity; i++ ) {
            toReturn.push_back( knot.t );
        }
    }

    return toReturn;
}

std::vector< double > BSpline2::fullKnots() const
{
    std::vector< double > knots = internalKnots();
    for( int i = 0; i < _degree; i++ ) {
        knots.insert( knots.begin(), 0 );
        knots.push_back( 1.0 );
    }
    return knots;
}

int BSpline2::numBezierCurves( bool includeDegenerates ) const
{
    if( includeDegenerates ) {
        return static_cast< int >( _controlPoints.size() ) - _degree;
    } else {
        const auto knots = fullKnotsNoMultiples();
        return static_cast< int >( knots.size() - 1 );
    }
}

std::array< BSpline2::UniquePtr, 2 > BSpline2::subdivide( double t ) const
{
    std::array< UniquePtr, 2 > toReturn;
    if( _controlPoints.size() == 0 || _controlPoints.size() <= _degree ) {
        return toReturn;
    }

    // This becomes a trivial operation if t is at the start or end: one of the returned splines is identical to the original
    // and the other is just a single point.
    if( mathUtility::closeEnough( t, 0 ) || t < 0 ) {
        const Control control( _degree + 1, _controlPoints[ 0 ] );
        toReturn[ 0 ] = createFromControlPoints( _degree, control );
        toReturn[ 1 ] = std::make_unique< Type >( *this );
        return toReturn;
    } else if ( mathUtility::closeEnough( t, 1.0 ) || t > 1.0 ) {
        toReturn[ 0 ] = std::make_unique< Type >( *this );
        const Control control( _degree + 1, _controlPoints.back() );
        toReturn[ 1 ] = createFromControlPoints( _degree, control );
        return toReturn;
    }

    const int numCurves = numBezierCurves( true );
    // Assume start/end t values of 0 and 1.
    std::vector< double > knots( _degree * 2 + numCurves - 1, 0.0 );
    for( int i = 0; i < _degree; i++ ) {
        knots[ knots.size() - 1 - i ] = 1.0;
    }

    // Keep track of the number of times 't' already appears in the knot vector.
    int numKnotsPlaced = 0;

    // Get the knots from GeoemtricTools. Only the intermediate knots are accessible. For instance, if a spline
    // comprises two curves, GeometricTools will only expose a single knot.
    const auto allInternalKnots = internalKnots();
    for( int i = 0; i < numCurves - 1; i++ ) {
        const auto knot = allInternalKnots[ i ];
        if( mathUtility::closeEnough( t, knot ) ) {
            numKnotsPlaced++;
        }
        knots[ _degree + i ] = knot;
    }

    // Where in the final knot vector the string of multiple knots for 't' will begin.
    int knotStringStartLoc = 0;
    while( knotStringStartLoc < knots.size() - 1
        && knots[ knotStringStartLoc ] < t
        && !mathUtility::closeEnough( t, knots[ knotStringStartLoc ] ) ) {
        knotStringStartLoc++;
    }

    // From here out, _controlPoints[i] has the knot vector { knots[i], knots[i+1]... knots[i+_degree-1] }.
    Control store = _controlPoints;

    // Insert 'knot' right before the knot with index 'i'. If there are already copies of 'knot', then 'i' should
    // index the first of them.
    auto insertKnot = [ & ]( int i, double knot, int numExistingCopies ) {
        // New control point to place right before the control point currently indexed by 'i'.
        Vector2 latestControlPoint = BSpline2Utility::interpFromControlPoints( i, knot, _degree, store, knots );

        // Duplicating the knot at 'i' invalidates _degree - 2 - ( numExistingCopies - 1 ) existing control points.
        const int numPrevToUpdate = _degree - 2 - ( numExistingCopies - 1 );
        if( numPrevToUpdate > 0 ) {
            std::vector< Vector2 > updatedPrevPoints( numPrevToUpdate );
            for( int j = 0; j < numPrevToUpdate; j++ ) {
                updatedPrevPoints[ j ] = BSpline2Utility::interpFromControlPoints(
                    i - _degree + 1 + numExistingCopies + j, knot, _degree, store, knots );
            }
            for( int j = 0; j < numPrevToUpdate; j++ ) {
                store[ i - _degree + 1 + numExistingCopies + j ] = updatedPrevPoints[ j ];
            }
        }

        store.insert( store.begin() + i, latestControlPoint );
        knots.insert( knots.begin() + i, knot );
    };

    while( numKnotsPlaced < _degree ) {
        insertKnot( knotStringStartLoc, t, numKnotsPlaced++ );
    }

    // Now 'store' contains all the necessary control points for the two splines; there remains only to come up with the right knot values
    // so that each curve spans its own [0,1] that maps correctly to the corresponding portion of the original spline.

    const Control firstControlPoints( store.begin(), store.begin() + knotStringStartLoc + 1 );
    std::vector< double > firstKnots( knots.begin() + _degree, knots.begin() + knotStringStartLoc );
    for( double& unnormalized : firstKnots ) {
        unnormalized /= t;
    }
    if( firstKnots.size() ) {
        toReturn[ 0 ] = createFromControlPointsAndKnots( _degree, firstControlPoints, firstKnots );
    } else {
        toReturn[ 0 ] = createFromControlPoints( _degree, firstControlPoints );
    }

    const Control secondControlPoints( store.begin() + knotStringStartLoc, store.end() );
    std::vector< double > secondKnots( knots.begin() + knotStringStartLoc + _degree, knots.end() - _degree );
    for( double& unnormalized : secondKnots ) {
        unnormalized = ( unnormalized - t ) / ( 1.0 - t );
    }
    if( secondKnots.size() ) {
        toReturn[ 1 ] = createFromControlPointsAndKnots( _degree, secondControlPoints, secondKnots );
    } else {
        toReturn[ 1 ] = createFromControlPoints( _degree, secondControlPoints );
    }

    return toReturn;
}

BSpline2::UniquePtr BSpline2::c0Copy( const Vector2& start, const Vector2& end ) const
{
    auto control = _controlPoints;
    control.front() = start;
    control.back() = end;
    return createFromControlPointsAndKnots( _degree, control, internalKnots() );
}

BSpline2::UniquePtr BSpline2::clone() const
{
    return std::make_unique< Type >( *this );
}

BSpline2::UniquePtr BSpline2::reverseCopy() const
{
    auto copied = clone();
    copied->reverse();
    return copied;
}

BSpline2::UniquePtr BSpline2::extractCurveForTInterval( const std::array< double, 2 >& interval ) const
{
    return extractCurveForTInterval( interval[ 0 ], interval[ 1 ] );
}

BSpline2::UniquePtr BSpline2::extractCurveForTInterval( double tStart, double tEnd ) const
{
    return extractCurveForTInterval( CurveInterval{ tStart, tEnd } );
}

BSpline2::UniquePtr BSpline2::extractCurveForTInterval( const CurveInterval& interval ) const
{    
    double tStart = interval.tStart();
    double tEnd = interval.tEnd();
    bool tIncreasing = interval.tIncreasing();
    const bool closedSpecialCase = tIncreasing != tStart <= tEnd;

    // Inside the function, let's have 'tStart' less than or equal to 'tEnd' and use 'reverse' to keep track of whether we flipped.
    bool reverse = false;
    if( tStart > tEnd ) {
        std::swap( tStart, tEnd );
        reverse = true;
        tIncreasing = !tIncreasing;
    }

    const bool easyStart = mathUtility::closeEnoughToZero( tStart ) || tStart < 0.0;
    const bool easyEnd = mathUtility::closeEnough( tEnd, 1.0 ) || tEnd > 1.0;

    UniquePtr toReturn;
    if( easyStart && easyEnd ) {
        toReturn = clone();
    } else if ( easyStart ) {
        auto split = subdivide( tEnd );
        toReturn = std::move( split[ tIncreasing ? 0 : 1 ] );
    } else if ( easyEnd ) {
        auto split = subdivide( tStart );
        toReturn = std::move( split[ tIncreasing ? 1 : 0 ] );
    } else {
        if( closedSpecialCase ) {
            // One of the situations where we're assuming the curve is closed and our interval is supposed to include T=0.
            auto split = subdivide( tStart );
            UniquePtr firstPiece = split[ 0 ]->reverseCopy();
            split = subdivide( tEnd );
            UniquePtr secondPiece = split[ 1 ]->reverseCopy();

            const std::vector< const Type* > parts{ firstPiece.get(), secondPiece.get() };
            toReturn = BSpline2Utility::stitchC0Spline( parts, defaultLengthPrecision );
        } else {
            const auto splitFirst = subdivide( tStart );
            const double newTEnd = ( tEnd - tStart ) / ( 1.0 - tStart );
            auto splitSecond = splitFirst[ 1 ]->subdivide( newTEnd );
            toReturn = std::move( splitSecond[ 0 ] );
        }
    }
    return reverse ? toReturn->reverseCopy() : std::move( toReturn );
}

std::vector< double > BSpline2::c0Times( const Type& spline )
{
    const auto knots = spline.internalKnots();
    if( spline.degree() == 1 ) {
        return knots;
    } else {
        std::vector< double > toReturn;

        const auto degree = spline.degree();
        size_t numMultiples = 0;
        for( size_t i = 1; i < knots.size(); i++ ) {
            const double& knot = knots[ i ];
            if( knot == 0 ) {
                continue;
            }
            if( knot >= 1.0 ) {
                break;
            }

            const double& prevKnot = knots[ i - 1 ];
            if( mathUtility::closeEnough( knot, prevKnot ) ) {
                numMultiples = ( numMultiples == 0 ? 2 : numMultiples + 1 );
            } else {
                numMultiples = 0;
            }

            if( numMultiples == degree ) {
                toReturn.push_back( knot );
            }
        }
        return toReturn;
    }
}

std::vector< double > BSpline2::naturalInterpolationChordLengthT( const std::vector< Vector2 >& passThrough )
{
    std::vector< double > distsSoFar;
    double totalDist = 0.0;
    if( passThrough.size() > 2 ) {
        for( size_t i = 0; i < passThrough.size() - 1; i++ ) {
            const double stepDist = ( passThrough[ i + 1 ] - passThrough[ i ] ).length();
            totalDist += stepDist;
            distsSoFar.push_back( totalDist );
        }
        std::vector< double > internalT;

        if( totalDist == 0 ) {
            for( size_t i = 1; i < passThrough.size() - 1; i++ ) {
                internalT.push_back( F_FROM_I( i, passThrough.size() ) );
            }
        } else {
            for( size_t i = 1; i < passThrough.size() - 1; i++ ) {
                internalT.push_back( distsSoFar[ i - 1 ] / totalDist );
            }
        }
        return internalT;
    } else {
        return {};
    }
}

std::unique_ptr< BSpline2 > BSpline2::naturalInterpolation( const std::vector< Vector2 >& passThrough )
{
    const auto internalT = naturalInterpolationChordLengthT( passThrough );
    return naturalInterpolation( passThrough, internalT );
}

std::unique_ptr< BSpline2 > BSpline2::naturalInterpolation(
    const std::vector< Vector2 >& passThrough, const std::vector< double >& internalT )
{
    const int degree = 3;

    if( passThrough.size() < degree ) {
        return polyline( { passThrough } );
    }

    if( internalT.size() != passThrough.size() - 2 ) {
        // I'm putting this here because it's too easy to forget to check the signature and
        // pass in full-length T vector that includes 0 at start and 1 at end. This will
        // case 0s to be passed as weights to the stitch-spline op at the end, which will yield
        // a spline with non-NaN control points that mysteriously produces NaN position values.
        // (All of this came up as a stupid detour while I was doing hatching shape-to-strokes.)
        throw std::runtime_error( "Wrong number of internal-T values passed to BSpline2::naturalInterpolation" );
    }

    const int numBeziers = static_cast< int >( passThrough.size() ) - 1;
    const int numUnknowns = numBeziers * 2; // Two internal controls for each cubic Bezier.
    using Matrix = Eigen::SparseMatrix< double >;
    Matrix A( numUnknowns, numUnknowns );
    // X coords, Y coords.
    std::vector< Eigen::VectorXd > b( 2 );
    for( size_t i = 0; i < 2; i++ ) {
        b[ i ].resize( numUnknowns );
    }

    using Triplet = Eigen::Triplet< double >;
    std::vector< Triplet > triplets;

    int rowIdx = 0;

    // Add the C1/C2 constraints
    for( int bezierIdx = 0; bezierIdx < numBeziers - 1; bezierIdx++ ) {
        // Our two splines are defined by control points a through g, where a, d, and g are
        // from 'passThrough', b and c are internal control points of curve 'bezierIdx' and
        // e and f are internal control points of curve 'bezierIdx'+1.

        const int bIdx = bezierIdx * 2;
        const int cIdx = bezierIdx * 2 + 1;
        const Vector2& dPoint = passThrough[ bezierIdx + 1 ];
        const int eIdx = ( bezierIdx + 1 ) * 2;
        const int fIdx = ( bezierIdx + 1 ) * 2 + 1;

/// If this is set, then when we calculate first and second derivatives in this system
/// of equations, we account for the relative sizes of Bezier curves' intended T-intervals.
//#define ACCOUNT_FOR_VARYING_T_INTERVALS
#ifdef ACCOUNT_FOR_VARYING_T_INTERVALS
        // We have to account for the possible difference in size of T-interval between
        // curves 'bezierIdx' and 'bezierIdx+1'. Call these tLength0 and tLength1, respectively.
        // If we don't account for this difference, the resulting curves will be only G1 and G2, not C1, and C2, _when
        // they are treated as though they cover the intended T-intervals and do not all individually span T in [0,1]_.
        const double tLength0 = internalT[ bezierIdx ] - ( bezierIdx == 0 ? 0.0 : internalT[ bezierIdx - 1 ] );
        const double tLength1 = ( bezierIdx == numBeziers - 2 ? 1.0 : internalT[ bezierIdx + 1 ] ) - internalT[ bezierIdx ];

        // C1: ( d - c ) / tLength0 = ( e - d ) / tLength1
        //     tLength1 * c + tLength0 * e = d * ( tLength0 + tLength1 )
        triplets.push_back( Triplet( rowIdx, cIdx, tLength1 ) );
        triplets.push_back( Triplet( rowIdx, eIdx, tLength0 ) );
        b[ 0 ]( rowIdx ) = ( tLength0 + tLength1 ) * dPoint.x();
        b[ 1 ]( rowIdx ) = ( tLength0 + tLength1 ) * dPoint.y();
        rowIdx++;

        // C2: ( d - 2c + b ) / tLength0 = ( f - 2e + d ) / tLength1
        //     tLength1 * b - 2 * tLength1 * c + 2 * tLength0 * e - tLength0 * f = d * ( tLength0 - tLength1 )
        triplets.push_back( Triplet( rowIdx, bIdx, tLength1 ) );
        triplets.push_back( Triplet( rowIdx, cIdx, -2.0 * tLength1 ) );
        triplets.push_back( Triplet( rowIdx, eIdx, 2.0 * tLength0 ) );
        triplets.push_back( Triplet( rowIdx, fIdx, -tLength0 ) );
        b[ 0 ]( rowIdx ) = ( tLength0 - tLength1 ) * dPoint.x();
        b[ 1 ]( rowIdx ) = ( tLength0 - tLength1 ) * dPoint.y();       
#else
        // C1: d - c = e - d
        //     c + e = 2d
        triplets.push_back( Triplet( rowIdx, cIdx, 1.0 ) );
        triplets.push_back( Triplet( rowIdx, eIdx, 1.0 ) );
        b[ 0 ]( rowIdx ) = 2.0 * dPoint.x();
        b[ 1 ]( rowIdx ) = 2.0 * dPoint.y();
        rowIdx++;

        // C2: d - 2c + b = f - 2e + d
        //     b - 2c + 2e - f = 0
        triplets.push_back( Triplet( rowIdx, bIdx, 1.0 ) );
        triplets.push_back( Triplet( rowIdx, cIdx, -2.0 ) );
        triplets.push_back( Triplet( rowIdx, eIdx, 2.0 ) );
        triplets.push_back( Triplet( rowIdx, fIdx, -1.0 ) );
        b[ 0 ]( rowIdx ) = 0.0;
        b[ 1 ]( rowIdx ) = 0.0;
#endif
        rowIdx++;
    }

    // Natural conditions: second derivative is 0 at both ends

    // For first curve:
    //  a - 2b + c = 0
    //  2b - c = a
    triplets.push_back( Triplet( rowIdx, 0, 2.0 ) );
    triplets.push_back( Triplet( rowIdx, 1, -1.0 ) );
    b[ 0 ]( rowIdx ) = passThrough.front().x();
    b[ 1 ]( rowIdx ) = passThrough.front().y();
    rowIdx++;

    // For last curve:
    // 2c - b = d
    triplets.push_back( Triplet( rowIdx, ( numBeziers - 1 ) * 2 + 1, 2.0 ) );
    triplets.push_back( Triplet( rowIdx, ( numBeziers - 1 ) * 2, -1.0 ) );
    b[ 0 ]( rowIdx ) = passThrough.back().x();
    b[ 1 ]( rowIdx ) = passThrough.back().y();

    A.setFromTriplets( triplets.begin(), triplets.end() );

    std::vector< Eigen::VectorXd > x( 2 );
    Eigen::SparseLU< Matrix > solver;
    solver.compute( A );
    for( size_t i = 0; i < 2; i++ ) {
        x[ i ] = solver.solve( b[ i ] );
    }

    std::vector< UniquePtr > beziers;    
    for( size_t i = 0; i < numBeziers; i++ ) {
        const Vector2& a = passThrough[ i ];
        const size_t bIdx = i * 2;
        const size_t cIdx = i * 2 + 1;
        const Vector2 b( x[ 0 ]( bIdx ), x[ 1 ]( bIdx ) );
        const Vector2 c( x[ 0 ]( cIdx ), x[ 1 ]( cIdx ) );
        const Vector2& d = passThrough[ i + 1 ];
        beziers.push_back( spline( degree, { a, b, c, d } ) );
    }

    if( numBeziers == 1 ) {
        return std::move( beziers.front() );
    } else {
        std::vector< double > tWeights( numBeziers );
        for( size_t i = 0; i < numBeziers; i++ ) {
            tWeights[ i ] = i == 0
                ? internalT[ i ]
                : ( i == numBeziers - 1 ? 1.0 - internalT.back() : internalT[ i ] - internalT[ i - 1 ] );
        }
        return BSpline2Utility::stitchC0Spline( uniquesToConstRaws( beziers ), tWeights, false );
    }

}

BSpline2::UniquePtr BSpline2::splineWithMultipleKnots(
    int degree, const Control& controlBefore, std::vector< int > knotsToDuplicate )
{
    auto beforeMultiples = createFromControlPoints( degree, controlBefore );
    Control control = beforeMultiples->controlPoints();
    std::vector< double > knots = beforeMultiples->fullKnots();

    // Double knot i+1 before knot i in order to keep indexing simpler.
    std::sort( knotsToDuplicate.begin(), knotsToDuplicate.end(), std::greater< int >() );
    for( const int intermediateKnotIndex : knotsToDuplicate ) {

        const int knotIndex = intermediateKnotIndex + degree;
        if( knotIndex >= knots.size() - degree ) {
            continue; // We don't duplicate end knots.
        }

        const double knotToDouble = knots[ knotIndex ];
        // Find out how many multiples already exist _and where the first one shows up_.
        int firstInstanceOf = knotIndex;
        int multiplicity = 0;
        for( int i = 0; i < (int)( knots.size() ); i++ ) {
            if( mathUtility::closeEnough( knotToDouble, knots[ i ] ) ) {
                if( i < firstInstanceOf ) {
                    firstInstanceOf = i;
                }
                multiplicity++;
            } else if ( knots[ i ] > knotToDouble ) {
                break;
            }
        }
        doubleKnot( firstInstanceOf, multiplicity, degree, knots, control );
    }

    const std::vector< double > intermediateKnotsOnly( knots.begin() + degree, knots.end() - degree );
    return createFromControlPointsAndKnots( degree, control, intermediateKnotsOnly );
}

std::vector< BSpline2::Control > BSpline2::breakIntoBCurves( std::vector< double >& storeTStarts ) const
{
    storeTStarts.clear();
    Control store;

    // Get the knot vector. This follows the format of Dr. Sederberg's book. So a spline which is
    // just a single degree-3 Bezier curve (with Bezier end conditions) will have the following knot vector:
    // [ 0 0 0 1 1 1 ]. Note that we don't include the extraneous end knots, according to Sederberg and in spite
    // of prevailing tradition.
    const int numCurves = numBezierCurves( true ); // Includes degenerate curves.
    // Assume start/end t values of 0 and 1.
    std::vector< double > knots = fullKnots();

    // From here out, _controlPoints[i] has the knot vector { knots[i], knots[i+1]... knots[i+_degree-1] }.
    store = _controlPoints;

    // Go through all the intermediate knots (all but the end knots) and multiply each one
    // to have multiplicity 'degree'. Do them in reverse order to make indexing easier.
    //
    // This breaks down the spline into its Bezier curves'
    // control points. Say, for instance, we start with a degree-3 spline with knot vector [ 0 0 0 0.25 0.5 1 1 1 ] (a spline
    // containing three curves). We will add multiple knots until the knot vector is [ 0 0 0 0.25 0.25 0.25 0.5 0.5 0.5 1 1 1 ], at which
    // point the corresponding points in 'store' can generate the curve from t=0 to t=0.25, the curve t=0.25 to t=0.5, and so on.
    const int numInitialIntermediateKnots = numCurves - 1;
    for( int interKnot = numInitialIntermediateKnots - 1; interKnot >= 0; interKnot-- ) {
        const int knotIdx = _degree + interKnot;

        // Determine how many multiples of this knot already exist.
        const double knotToDouble = knots[ knotIdx ];
        int multiplicity = 0;
        int firstOccurrence = knotIdx;
        for( int j = knotIdx - _degree; j < knotIdx + _degree; j++ ) {
            if( mathUtility::closeEnough( knotToDouble, knots[ j ] ) ) {
                multiplicity++;
                if( j < firstOccurrence ) {
                    firstOccurrence = j;
                }
            }
        }

        while( multiplicity < _degree ) {
            doubleKnot( firstOccurrence, multiplicity++, _degree, knots, store );
        }

        // In the (unusual) event that 'multiplicity' is greater than '_degree', delete the
        // extra knots. This makes the immediately following code less of a hassle.
        while( multiplicity-- > _degree ) {
            knots.erase( knots.begin() + firstOccurrence );
            store.erase( store.begin() + firstOccurrence );
        }
    }

    // Now 'store' contains all the control points; split them into per-Bezier vectors for the client's use.
    const int numCurvesExcludingDegenerate = static_cast< int >( knots.size() ) / _degree - 1; // num unique Bezier endpoints minus 1.
    std::vector< Control > toReturn( numCurvesExcludingDegenerate );
    storeTStarts = std::vector< double >( numCurvesExcludingDegenerate );
    for( int c = 0; c < numCurvesExcludingDegenerate; c++ ) {
        storeTStarts[ c ] = knots[ _degree * c ];
        toReturn[ c ] = Control(
            store.begin() + _degree * c,
            store.begin() + _degree * ( c + 1 ) + 1 );
    }
    return toReturn;
}

BSpline2::UniquePtr BSpline2::offset( bool whichDirection, double offset ) const
{
    const size_t numPointsToUse = numBezierCurves( true ) * 10;

    std::vector< Vector2 > samples( numPointsToUse );
    for( size_t i = 0; i < numPointsToUse; i++ ) {
        const double t = static_cast< double >( i ) / static_cast< double >( numPointsToUse - 1 );
        auto pos = position( t );
        auto dir = derivative( t );
        dir.turnPerpendicular();
        dir.normalize();
        pos += dir * offset * ( whichDirection ? 1.0 : -1.0 );
        samples[ i ] = pos;
    }
    return createFitToDataPoints( degree(), static_cast< int >( controlPoints().size() ), samples );
}

BSpline2::UniquePtr BSpline2::createFitToDataOrFallback(
    int numCurves,
    const Control& dataPoints,
    CurveFitParametrizeType parametrize )
{
    const auto numSamples = dataPoints.size();
    for( int degree = 3; degree > 1; degree-- ) {
        if( numSamples >= numCurves + 2 * degree + 1 ) {
            return createFitToDataPoints( degree, numCurves + degree, dataPoints, parametrize );
        }
        // What if we reduce the number of curves.
        const int maxNumCurves = static_cast< int >( numSamples ) - 2 * degree - 1;
        if( maxNumCurves > 0 ) {
            return createFitToDataPoints( degree, maxNumCurves + degree, dataPoints, parametrize );
        }
    }

    switch( numSamples ) {
    case 0: {
        throw BuildSplineException( "Zero data points provided." );
    }
    case 1: {
        return createFromControlPoints( 1, { dataPoints.front(), dataPoints.front() } );
    }
    case 2: {
        return createFromControlPoints( 1, { dataPoints.front(), dataPoints.back() } );
    }
    case 3:
    case 4: {
        return createFromControlPoints( static_cast< int >( numSamples - 1 ), dataPoints );
    }
    case 5: {
        return createFromControlPoints( 3, dataPoints );
    }
    case 6:
    case 7: {
        return createFitToDataPoints( 2, 3, dataPoints, parametrize );
    }
    default: {
        throw BuildSplineException( "Should not have reached." );
    }
    }
}

BSpline2::UniquePtr BSpline2::createFromControlPoints( int degree, const Control& controlPoints )
{
    return std::make_unique< Type >( degree, controlPoints );
}

BSpline2::UniquePtr BSpline2::createFromControlPointsAndKnots(
    int degree, const Control& controlPoints, const std::vector< double >& intermediateKnots )
{
    return std::make_unique< Type >( degree, controlPoints, intermediateKnots );
}

BSpline2::Control BSpline2::controlFitToDataPoints(
    int degree,
    int numControlPoints,
    const Control& dataPoints,
    CurveFitParametrizeType parametrize )
{
    const size_t numSamples = dataPoints.size();
    if( numSamples == 0 ) {
        throw BuildSplineException( "Zero data points provided." );
    }

    double* pTimes = nullptr; // null means use GTE's default mechanism.
    std::vector< double > sampleTimes( numSamples ); // For when I don't want to use the default mechanism.
    switch( parametrize ) {
    case CurveFitParametrizeType::ChordLength: {
        // Find all chord lengths
        double* chordLengths = new double[ numSamples - 1 ];
        double chordLengthSum = 0;
        for( int chord = 0; chord < numSamples - 1; chord++ )
        {
            const auto chordLength = ( dataPoints[ chord ] - dataPoints[ chord + 1 ] ).length();
            chordLengths[ chord ] = chordLength;
            chordLengthSum += chordLength;
        }

        if( chordLengthSum > 0 ) {
            sampleTimes.front() = 0;
            double running=0;
            for( int i = 1; i < numSamples - 1; i++ )
            {
                running += chordLengths[ i - 1 ];
                double sampleTime = running / chordLengthSum;
                sampleTimes[ i ] = sampleTime;
            }
            sampleTimes.back() = 1.0;
        } else {
            // Revert to uniform T intervals.
            for( int i = 0; i < numSamples; i++ ) {
                sampleTimes[ i ] = static_cast< double >( i ) / static_cast< double >( numSamples - 1 );
            }
        }

        delete[] chordLengths;
        pTimes = &sampleTimes[ 0 ];
        break;
    }
    case CurveFitParametrizeType::UseXAsT: {
        for( int i = 0; i < numSamples; i++ ) {
            sampleTimes[ i ] = dataPoints[ i ].x();
        }
        pTimes = &sampleTimes[ 0 ];
        break;
    }
    }

    // Convert sample points to GTE form
    double* arrayOfVals = new double[ dataPoints.size() * 2 ];
    size_t index=0;
    for( auto it = dataPoints.cbegin(); it != dataPoints.cend(); it++ )
    {
        const auto& toCopy = *it;
        arrayOfVals[ index++ ] = toCopy.x();
        arrayOfVals[ index++ ] = toCopy.y();
    }

    gte::BSplineCurveFit fitter(
        2,
        static_cast< int32_t >( numSamples ),
        arrayOfVals,
        static_cast< int32_t >( degree ),
        static_cast< int32_t >( numControlPoints ),
        pTimes );

    // Now get the control points from the fitter.  This is all assuming that the
    // curve fitter internally devised a uniform, open-ended, Bezier end condition spline.
    const double* fitterControlData = fitter.GetControlData();
    Control controlPoints( numControlPoints );
    for(int i = 0; i < numControlPoints; i++ )
    {
        controlPoints[ i ] = Vector2(
                    *( fitterControlData + i * 2 ),
                    *( fitterControlData + i * 2 + 1 ) );
    }
    delete[] arrayOfVals;
    return controlPoints;
}

BSpline2::UniquePtr BSpline2::createFitToDataPoints(
    int degree,
    int numControlPoints,
    const std::vector<Vector2>& dataPoints,
    CurveFitParametrizeType parametrize )
{    
    const auto control = controlFitToDataPoints( degree, numControlPoints, dataPoints, parametrize );
    return createFromControlPoints( degree, control );
}

} // core
