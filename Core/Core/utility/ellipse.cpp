#include <utility/ellipse.h>

#include <utility/bspline2.h>
#include <utility/bspline2utility.h>
#include <utility/casts.h>
#include <utility/mathutility.h>

#include <boost/math/constants/constants.hpp>

namespace core {

namespace {

const int splineDegree = 3;
const double kappa = 0.5522847498;

const std::vector< Vector2 > unitCircleControl
{
    Vector2( 1.0, 0 ),
    Vector2( 1.0, kappa ),
    Vector2( kappa, 1.0 ),
    Vector2( 0, 1.0 ),
    Vector2( -kappa, 1.0 ),
    Vector2( -1.0, kappa ),
    Vector2( -1.0, 0 ),
    Vector2( -1.0, -kappa ),
    Vector2( -kappa, -1.0 ),
    Vector2( 0, -1.0 ),
    Vector2( kappa, -1.0 ),
    Vector2( 1.0, -kappa )
};

void rotatePointsCounterclockwise( std::vector< Vector2 >& points, double radians )
{
    const double cosRadians = std::cos( radians );
    const double sinRadians = std::sin( radians );

    for( auto& p : points ) {
        p = Vector2(
            p.x() * cosRadians - p.y() * sinRadians,
            p.x() * sinRadians + p.y() * cosRadians );
    }
}

} // unnamed

Ellipse::Parametric Ellipse::circle( const Vector2& center, double radius )
{
    Ellipse::Parametric toReturn;
    toReturn.a = radius;
    toReturn.b = radius;
    toReturn.center = center;
    toReturn.tauCounterclockwise = 0.0;
    return toReturn;
}

bool Ellipse::Parametric::valid() const
{
    const auto badNum = []( const double& n )
    {
        return std::isinf( n ) || std::isnan( n );
    };

    return !badNum( a )
        && !badNum( b )
        && !badNum( center.x() )
        && !badNum( center.y() )
        && !badNum( tauCounterclockwise );
}

Vector2 Ellipse::Parametric::pos( double radians ) const
{
    const double cosRadians = std::cos( radians );
    const double sinRadians = std::sin( radians );
    const double cosTau = std::cos( tauCounterclockwise );
    const double sinTau = std::sin( tauCounterclockwise );
    return Vector2
    {
        center.x() + cosTau * a * cosRadians - sinTau * b * sinRadians,
        center.y() + sinTau * a * cosRadians + cosTau * b * sinRadians
    };
}

std::unique_ptr< BSpline2 > Ellipse::Parametric::splineApprox( const PolarInterval& angleInterval ) const
{
    // From https://www.tinaja.com/glib/bezcirc2.pdf

    const double theta = angleInterval.length();
    const double halfPi = boost::math::constants::pi< double >() * 0.5;
    const double quotient = theta / halfPi;
    const size_t numQuadrants = static_cast< size_t >( std::floor( quotient ) );
    if( angleInterval.full() || numQuadrants > 3 ) {
        return splineApprox();
    }
    const double smallRads = std::fmod( theta, halfPi );

    std::vector< Vector2 > control;

    // Start out pretending we're dealing with a unit circle and the arc starts at 0 degrees.
    {
        if( numQuadrants > 0 ) {
            control.insert( control.begin(), unitCircleControl.begin(), unitCircleControl.begin() + 1 + numQuadrants * 3 );
        }

        // Set up the less-than-90-degree arc remaining
        std::vector< Vector2 > smallControl( 4 );
        smallControl[ 0 ] = Vector2( std::cos( smallRads / 2.0 ), std::sin( smallRads / -2.0 ) );
        if( !mathUtility::closeEnoughToZero( smallControl[ 0 ].y() ) ) {
            smallControl[ 3 ] = Vector2( smallControl[ 0 ].x(), smallControl[ 0 ].y() * -1.0 );
            smallControl[ 1 ] = Vector2(
               ( 4.0 - smallControl[ 0 ].x() ) / 3.0,
               ( ( 1.0 - smallControl[ 0 ].x() ) * ( 3.0 - smallControl[ 0 ].x() ) ) / ( 3.0 * smallControl[ 0 ].y() )
                        );
            smallControl[ 2 ] = Vector2( smallControl[ 1 ].x(), smallControl[ 1 ].y() * -1.0 );

            // Rotate 'smallControl' to line up with the start of the quadrant it's supposed to fit in.
            rotatePointsCounterclockwise( smallControl, smallRads / 2.0 + halfPi * static_cast< double >( numQuadrants ) );

            if( numQuadrants > 0 ) {
                control.insert( control.end(), smallControl.begin() + 1, smallControl.end() );
            } else {
                control = smallControl;
            }

            // Rotate all the control points
            rotatePointsCounterclockwise( control, angleInterval.minRadians() );
        }
    }

    // Angle might have been too small.
    if( control.size() > 0 ) {
        unitCircleToEllipse( control );
        const size_t numBeziers = ( control.size() - 1 ) / 3;
        std::vector< std::unique_ptr< BSpline2 > > beziers( numBeziers );
        for( size_t i = 0; i < numBeziers; i++ ) {
            const size_t firstControl = i * 3;
            beziers[ i ] = BSpline2::spline( splineDegree, { control.begin() + firstControl, control.begin() + firstControl + 4 } );
        }       

        std::vector< double > tWeights( numQuadrants, 1.0 );
        tWeights.push_back( smallRads / halfPi );
        auto toReturn = BSpline2Utility::stitchC0Spline( uniquesToConstRaws< BSpline2 >( beziers ), tWeights );
        return toReturn;
    } else {
        return nullptr;
    }
}

std::unique_ptr< BSpline2 > Ellipse::Parametric::splineApprox() const
{
    // From http://www.whizkidtech.redprince.net/bezier/circle/

    // First point is at normalized position (1,0) fourth at (0,1), seventh at (-1,0).
    std::vector< Vector2 > control = unitCircleControl;
    unitCircleToEllipse( control );

    std::vector< std::unique_ptr< BSpline2 > > beziers( 4 );
    beziers[ 0 ] = BSpline2::spline( splineDegree, { control.begin(), control.begin() + 4 } );
    beziers[ 1 ] = BSpline2::spline( splineDegree, { control.begin() + 3, control.begin() + 7 } );
    beziers[ 2 ] = BSpline2::spline( splineDegree, { control.begin() + 6, control.begin() + 10 } );
    beziers[ 3 ] = BSpline2::spline( splineDegree, { control[ 9 ], control[ 10 ], control[ 11 ], control[ 0 ] } );

    auto toReturn = BSpline2Utility::stitchC0Spline( uniquesToConstRaws< BSpline2 >( beziers ), std::vector< double >( 4, 0.25 ) );
    return toReturn;
}

void Ellipse::Parametric::unitCircleToEllipse( std::vector< Vector2 >& control ) const
{
    // Scale, rotate and translate.
    const double cosTau = std::cos( tauCounterclockwise );
    const double sinTau = std::sin( tauCounterclockwise );
    for( auto& p : control ) {
        p.setX( p.x() * a );
        p.setY( p.y() * b );
        p = Vector2(
            p.x() * cosTau - p.y() * sinTau,
            p.x() * sinTau + p.y() * cosTau )
            + center;
    }
}

double Ellipse::Parametric::angle( const Vector2& pos, Vector2& unitCircleSpace ) const
{
    // Imagine that 'pos' is a position on the ellipse (it doesn't need to be, but this makes thinking through the process easier).
    // Move from world space to unit-circle space.

    // Undo translation.
    unitCircleSpace = pos - center;

    // Undo rotation
    const double cosTau = std::cos( -tauCounterclockwise );
    const double sinTau = std::sin( -tauCounterclockwise );
    unitCircleSpace = Vector2(
        unitCircleSpace.x() * cosTau - unitCircleSpace.y() * sinTau,
        unitCircleSpace.x() * sinTau + unitCircleSpace.y() * cosTau );

    // Undo scale.
    unitCircleSpace = Vector2(
        unitCircleSpace.x() / a,
        unitCircleSpace.y() / b );

    return std::atan2( unitCircleSpace.y(), unitCircleSpace.x() );
}

Ellipse::Conic Ellipse::conic( const Ellipse::Parametric& param )
{
    const double cosTau = std::cos( param.tauCounterclockwise );
    const double sinTau = std::sin( param.tauCounterclockwise );

    Ellipse::Conic conic;
    // From https://www.cs.cornell.edu/cv/OtherPdf/Ellipse.pdf (18)
    conic.a = pow( param.b * cosTau, 2 ) + pow( param.a * sinTau, 2 );
    conic.b = -2.0 * cosTau * sinTau * ( param.a * param.a - param.b * param.b );
    conic.c = pow( param.b * sinTau, 2 ) + pow( param.a * cosTau, 2 );
    conic.d = -2.0 * conic.a * param.center.x() - param.center.y() * conic.b;
    conic.e = -2.0 * conic.c * param.center.y() - param.center.x() * conic.b;
    conic.f = -1.0 * pow( param.a * param.b, 2 )
        + conic.a * param.center.x() * param.center.x()
        + conic.b * param.center.x() * param.center.y()
        + conic.c * param.center.y() * param.center.y();
    return conic;
}

} // core
