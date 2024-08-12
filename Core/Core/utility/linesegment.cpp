#include <utility/linesegment.h>
#include <utility/mathutility.h>
#include <utility/twodarray.h>

namespace core {

namespace {

IntCoord intCoordForRasterizing( double fX, double fY )
{
    return IntCoord(
        static_cast< int >( std::floor( fX ) ),
        static_cast< int >( std::floor( fY ) ) );
}

IntCoord intCoordForRasterizing( const Vector2& v )
{
    return intCoordForRasterizing( v.x(), v.y() );
}

} // unnamed

LineSegment::LineSegment()
{
}

LineSegment::LineSegment( const Vector2& aArg, const Vector2& bArg ) : a( aArg ), b( bArg )
{
}

LineSegment LineSegment::reverse() const
{
    return { b, a };
}

Vector2 LineSegment::pos( double t ) const
{
    return mathUtility::lerp< Vector2 >( a, b, t );
}

void LineSegment::extend( double moveABackBy, double moveBAheadBy )
{
    Vector2 vec = b - a;
    const auto length = vec.length();
    vec.normalize();

    b = a + vec * ( length + moveBAheadBy );
    a = a - vec * moveABackBy;
}

Vector2 LineSegment::asVec() const
{
    return b - a;
}

double LineSegment::length() const
{
    return ( b - a ).length();
}

Vector2 LineSegment::midpoint() const
{
    return ( a + b ) * 0.5;
}

BoundingBoxd LineSegment::bounds() const
{
    return { a, b };
}

std::vector< IntCoord > rasterizeSegment_midpoint( const Vector2& a, const Vector2& b )
{
    auto aI = intCoordForRasterizing( a );
    auto bI = intCoordForRasterizing( b );

    return rasterizeSegment_midpoint( aI, bI );
}

std::vector< IntCoord > rasterizeSegment_midpoint( const IntCoord& a, const IntCoord& b )
{
    // From https://www.geeksforgeeks.org/mid-point-line-generation-algorithm/?ref=rp

    // d(x) is 0 on line, <0 below line, and >0 above line, in the canonical case of x1<x2, dy < dx.

    const auto dX = b.x() - a.x();
    const auto dY = b.y() - a.y();

    std::vector< IntCoord > toReturn{ a };

    const bool negativeSlope = dX * dY < 0;
    const int xStep = dX < 0 ? -1 : 1;
    const int yStep = dY < 0 ? -1 : 1;

    const auto x1 = a.x();
    const auto x2 = b.x();
    const auto y1 = a.y();
    const auto y2 = b.y();

    auto x = x1;
    auto y = y1;

    if( y1 == y2 ) {
        while( x != x2 ) {
            x += xStep;
            toReturn.push_back( IntCoord( x, y1 ) );
        }
        return toReturn;
    } else if( x1 == x2 ) {
        while( y != y2 ) {
            y += yStep;
            toReturn.push_back( IntCoord( x1, y ) );
        }
        return toReturn;
    }

    if( abs( dX ) > abs( dY ) ) {
        auto d = dY * xStep - dX * yStep / 2;
        while( x != x2 ) {
            x += xStep;
            if( ( d < 0 ) ^ negativeSlope ) {
                d += dY * xStep;
            } else {
                d += dY * xStep - dX * yStep;
                y += yStep;
            }
            toReturn.push_back( IntCoord( x, y ) );
        }
    } else {
        auto d = dX * yStep - dY * xStep / 2;
        while( y != y2 ) {
            y += yStep;
            if( ( d < 0 ) ^ negativeSlope ) {
                d += dX * yStep;
            } else {
                d += dX * yStep - dY * xStep;
                x += xStep;
            }
            toReturn.push_back( IntCoord( x, y ) );
        }
    }

    return toReturn;
}

std::vector< IntCoord > rasterizeSegment_Bresenham( const Vector2& a, const Vector2& b )
{
    return rasterizeSegment_Bresenham( intCoordForRasterizing( a ), intCoordForRasterizing( b ) );
}

std::vector< IntCoord > rasterizeSegment_Bresenham( const IntCoord& a, const IntCoord& b )
{
    std::vector< IntCoord > toReturn;

    // From https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    const int xStart = a.x(), xEnd = b.x(), yStart = a.y(), yEnd = b.y();
    const int xDelta = xEnd - xStart;
    const int yDelta = yEnd - yStart;
    const int xInc = xStart < xEnd ? 1 : -1;
    const int yInc = yStart < yEnd ? 1 : -1;
    if( xDelta == 0 && yDelta == 0 ) {
        toReturn.push_back( IntCoord( xStart, yStart ) );
    } else if( xDelta == 0 ) {
        const int yEndExc = yEnd + yInc;
        for( int y = yStart; y != yEndExc; y+=yInc ) {
            toReturn.push_back( IntCoord( xStart, y ) );
        }
    } else if( yDelta == 0 ) {
        const int xEndExc = xEnd + xInc;
        for( int x = xStart; x != xEndExc; x+=xInc ) {
            toReturn.push_back( IntCoord( x, yStart ) );
        }
    } else if( abs( xDelta ) > abs( yDelta ) ) {
        const double deltaErr = abs( boost::numeric_cast< double >( yDelta ) / boost::numeric_cast< double >( xDelta ) );
        double error = deltaErr - 0.5;
        int y = yStart;
        const int xEndExc = xEnd + xInc;
        for( int x = xStart; x != xEndExc; x+=xInc ) {
            toReturn.push_back( IntCoord( x, y ) );
            error += deltaErr;
            if( error >= 0.5 ) {
                y += yInc;
                error--;
            }
        }
    } else {
        const double deltaErr = abs( boost::numeric_cast< double >( xDelta ) / boost::numeric_cast< double >( yDelta ) );
        double error = deltaErr - 0.5;
        int x = xStart;
        const int yEndExc = yEnd + yInc;
        for( int y = yStart; y != yEndExc; y+=yInc ) {
            toReturn.push_back( IntCoord( x, y ) );
            error += deltaErr;
            if( error >= 0.5 ) {
                x += xInc;
                error--;
            }
        }
    }

    return toReturn;
}

std::vector< IntCoord > rasterizeSegment_floatingPoint( const Vector2& a, const Vector2& b )
{
    std::vector< IntCoord > toReturn;

    const auto aToB = b - a;
    const auto dX = aToB.x();
    const auto dY = aToB.y();

    // Exit cases
    {
        const auto aI = intCoordForRasterizing( a );
        const auto bI = intCoordForRasterizing( b );
        if( aI == bI ) {
            return { aI };
        }
    }

    double x = a.x();
    double y = a.y();
    bool done = false;

    if( std::fabs( dX ) > std::fabs( dY ) ) {
        double xTravel = 0.0;
        const auto xStep = dX < 0 ? -1.0 : 1.0;
        while( true ) {
            y = a.y() + dY * xTravel / std::fabs( dX );
            if( done ) {
                const auto maybeLast = intCoordForRasterizing( x, y );
                if( maybeLast != toReturn.back() ) {
                    toReturn.push_back( maybeLast );
                }
                break;
            }
            toReturn.push_back( intCoordForRasterizing( x, y ) );
            const auto amountLeft = std::fabs( dX ) - xTravel;
            if( amountLeft > 1.0 ) {
                x += xStep;
                xTravel += 1.0;
            } else {
                x = b.x();
                xTravel = std::fabs( dX );
                done = true;
            }
        }
    } else {
        double yTravel = 0.0;
        const auto yStep = dY < 0 ? -1.0 : 1.0;
        while( true ) {
            x = a.x() + dX * yTravel / std::fabs( dY );
            if( done ) {
                const auto maybeLast = intCoordForRasterizing( x, y );
                if( maybeLast != toReturn.back() ) {
                    toReturn.push_back( maybeLast );
                }
                break;
            }
            toReturn.push_back( intCoordForRasterizing( x, y ) );
            const auto amountLeft = std::fabs( dY ) - yTravel;
            if( amountLeft > 1.0 ) {
                y += yStep;
                yTravel += 1.0;
            } else {
                y = b.y();
                yTravel = std::fabs( dY );
                done = true;
            }
        }
    }

    return toReturn;
}

LineSegment LineSegment::fromPosAndDir( const Vector2& pos, const Vector2& dir )
{
    return LineSegment( pos, pos + dir );
}

double LineSegment::t( const Vector2& p ) const
{
    const auto len = length();
    if( mathUtility::closeEnoughToZero( len ) ) {
        return 0.;
    } else {
        return std::clamp( ( p - a ).length() / len, 0., 1. );
    }
}

} // core
