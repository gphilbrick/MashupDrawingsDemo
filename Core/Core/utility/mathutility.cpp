#include <utility/mathutility.h>

#include <utility/boundingbox.h>
#include <utility/bspline2.h>
#include <utility/linesegment.h>
#include <utility/vector2.h>
#include <utility/vector2.h>
#include <utility/vector3.h>

#include <boost/math/constants/constants.hpp>

#include <cstdlib>

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

namespace core {
namespace mathUtility {

using Polygon = std::vector< Vector2 >;
using Polygons = std::vector< Polygon >;

namespace {

// See http://geomalgorithms.com/a03-_inclusion.html for explanation of
// which edges and edge-endpoints are included.
bool hitsSegment( const Vector2& point, const Vector2& a, const Vector2& b )
{
    const bool yAlmostEqual = closeEnough( a.y(), b.y() );
    if( yAlmostEqual ) {
        return false;
    } else {
        const double yMin = std::min( a.y(), b.y() );
        const double yMax = std::max( a.y(), b.y() );
        if( point.y() >= yMin && point.y() < yMax ) {
            const double xCutoff =
                a.x() + ( b.x() - a.x() ) * ( point.y() - a.y() ) / ( b.y() - a.y() );
            return point.x() < xCutoff;
        } else {
            return false;
        }
    }
};

} // unnamed

Vector3 nearestPointOnSegmentToLine( const Vector3& segA, const Vector3& segB, const Vector3& lineA, const Vector3& lineB )
{
    // From John Alexiou's answer at https://math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d
    auto segDir = segB - segA;
    segDir.normalize();
    auto lineDir = lineB - lineA;
    lineDir.normalize();

    // This is the direction of the shortest-distance connection between lines A and B. It will be 0 if
    // lines A and B are parallel (in which case I think we'll just end up returning 'b1', which is
    // appropriate).
    auto crossDir = Vector3::cross( lineDir, segDir );
    const auto crossDirLengthSquared = Vector3::dot( crossDir, crossDir );
    if( closeEnoughToZero( crossDirLengthSquared ) ) {
        return segA;
    }

    auto tSeg = Vector3::dot( Vector3::cross( lineDir, crossDir ), segA - lineA ) / crossDirLengthSquared;
    tSeg = std::clamp( tSeg, 0., ( segB - segA ).length() );
    return segA + segDir * tSeg;
}

Vector3 nearestPointOnOtherLine( const Vector3& a1, const Vector3& a2, const Vector3& b1, const Vector3& b2 )
{
    // From John Alexiou's answer at https://math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d
    auto aDir = a2 - a1;
    aDir.normalize();
    auto bDir = b2 - b1;
    bDir.normalize();

    // This is the direction of the shortest-distance connection between lines A and B. It will be 0 if
    // lines A and B are parallel (in which case I think we'll just end up returning 'b1', which is
    // appropriate).
    auto crossDir = Vector3::cross( aDir, bDir );
    const auto crossDirLengthSquared = Vector3::dot( crossDir, crossDir );
    if( closeEnoughToZero( crossDirLengthSquared ) ) {
        return b1;
    }

    const auto tB = Vector3::dot( Vector3::cross( aDir, crossDir ), b1 - a1 ) / crossDirLengthSquared;
    return b1 + bDir * tB;
}

Vector3 rayIntersectPlane( const Vector3& rayA, const Vector3& rayB, const Vector3& onPlane, const Vector3& planeNormal, bool& valid )
{
    const auto denominator = Vector3::dot( rayB - rayA, planeNormal );
    if( std::fabs( denominator ) < 1e-6 ) {
        valid = false;
        return Vector3{ 0, 0, 0 };
    }
    valid = true;
    const auto numerator = Vector3::dot( onPlane - rayA, planeNormal );
    const auto t = numerator / denominator;
    if( t < 0 ) {
        valid = false;
        return Vector3{ 0, 0, 0 };
    } else {
        return lerp( rayA, rayB, t );
    }
}

Vector3 projectPointToPlane( const Vector3& point, const Vector3& onPlane, const Vector3& planeNormal )
{
    return point - planeNormal * ( Vector3::dot( planeNormal, point - onPlane ) );
}

Vector2 rotate( const Vector2& toRotate, const Vector2& about, double cosRadians, double sinRadians )
{
    auto rel = toRotate - about;

    const double x = cosRadians * rel.x() - sinRadians * rel.y();
    const double y = sinRadians * rel.x() + cosRadians * rel.y();

    return Vector2{ x, y } + about;
}

Vector2 rotate( const Vector2& toRotate, const Vector2& about, double radians )
{
    return rotate( toRotate, about, std::cos( radians ), std::sin( radians ) );
}

boost::optional< LineSegment > keepRayInBox( const Vector2& rayA, const Vector2& rayB, const BoundingBoxd& box )
{
    using Pos = Vector2;
    std::vector< Pos > hits;

    using OptPos = boost::optional< Pos >;

    // I know there's a nice, trigonometric way to do this, but I'm feeling lazy.

    OptPos hit1 = rayLineSegmentIntersection( rayA, rayB, box.topLeft(), box.bottomLeft() );
    if( hit1.is_initialized() ) {
        hits.push_back( hit1.value() );
    }

    OptPos hit2 = lineLineSegmentIntersection( rayA, rayB, box.topRight(), box.bottomRight() );
    if( hit2.is_initialized() ) {
        hits.push_back( hit2.value() );
    }

    OptPos hit3 = lineLineSegmentIntersection( rayA, rayB, box.topLeft(), box.topRight() );
    if( hit3.is_initialized() ) {
        hits.push_back( hit3.value() );
    }

    OptPos hit4 = lineLineSegmentIntersection( rayA, rayB, box.bottomLeft(), box.bottomRight() );
    if( hit4.is_initialized() ) {
        hits.push_back( hit4.value() );
    }

    if( box.contains( rayA ) ) {
        if( hits.size() > 0 ) {
            return LineSegment{ rayA, hits.front() };
        } else {
            // Shouldn't ever happen?
            return LineSegment{ rayA, rayA };
        }
    } else {
        if( hits.size() == 2 ) {
            return LineSegment{ hits.front(), hits.back() };
        } else {
            if( hit1.is_initialized() && hit2.is_initialized() ) {
                return LineSegment{ hit1.value(), hit2.value() };
            } else if( hit3.is_initialized() && hit4.is_initialized() ) {
                return LineSegment{ hit3.value(), hit4.value() };
            } else {
                // Yeah, I know, this is embarrassing.
                return boost::none;
            }
        }
    }
}

boost::optional< LineSegment > keepLineInBox( const LineSegment& line, const BoundingBoxd& box )
{
    using Pos = Vector2;
    std::vector< Pos > hits;

    using OptPos = boost::optional< Pos >;

    // I know there's a nice, trigonometric way to do this, but I'm feeling lazy.

    OptPos hit1 = lineLineSegmentIntersection( line.a, line.b, box.topLeft(), box.bottomLeft() );
    if( hit1.is_initialized() ) {
        hits.push_back( hit1.value() );
    }

    OptPos hit2 = lineLineSegmentIntersection( line.a, line.b, box.topRight(), box.bottomRight() );
    if( hit2.is_initialized() ) {
        hits.push_back( hit2.value() );
    }

    OptPos hit3 = lineLineSegmentIntersection( line.a, line.b, box.topLeft(), box.topRight() );
    if( hit3.is_initialized() ) {
        hits.push_back( hit3.value() );
    }

    OptPos hit4 = lineLineSegmentIntersection( line.a, line.b, box.bottomLeft(), box.bottomRight() );
    if( hit4.is_initialized() ) {
        hits.push_back( hit4.value() );
    }

    if( hits.size() == 2 ) {
        return LineSegment{ hits.front(), hits.back() };
    } else {
        if( hit1.is_initialized() && hit2.is_initialized() ) {
            return LineSegment{ hit1.value(), hit2.value() };
        } else if( hit3.is_initialized() && hit4.is_initialized() ) {
            return LineSegment{ hit3.value(), hit4.value() };
        } else {
            // Yeah, I know, this is embarrassing.
            return boost::none;
        }
    }
}

boost::optional< LineSegment > keepLineInBox( const LineSegment& line, const Vector2& cornerA, const Vector2& cornerB )
{
    return keepLineInBox( line, BoundingBoxd( cornerA, cornerB ) );
}

// https://en.wikipedia.org/wiki/Smoothstep
double smoothstep( const double edge0, const double edge1, const double x )
{
  const double xClamped = std::min< double >( std::max< double >( ( x - edge0 ) / ( edge1 - edge0 ), 0.0 ), 1.0 );
  return xClamped * xClamped * ( 3.0f - 2.0f * xClamped );
}

bool polygonsDoNotIntersect( const std::vector< Vector2 >& a, const std::vector< Vector2 >& b )
{
    const auto vtxOfAInsideB = []( const Polygon& a, const Polygon& b ) -> bool
    {
        const Polygons bPolys{ b };
        for( const auto& fromA : a ) {
            if( insidePolygons( fromA, bPolys ) ) {
                return true;
            }
        }
        return false;
    };

    if( vtxOfAInsideB( a, b ) ) {
        return false;
    }
    if( vtxOfAInsideB( b, a ) ) {
        return false;
    }

    // No vertex of 'a' is inside 'b', and no vertex of 'b' is inside 'a'. But it is still possible that
    // a segment of 'a' intersects 'b' or a segment of 'b' intersects 'a'.
    if( a.size() < 2 || b.size() < 2 ) {
        return true;    
    }

    Vector2 unused;

    for( size_t i = 0; i < a.size(); i++ ) {
        const size_t j = ( i + 1 ) % a.size();
        const auto& aStart = a[ i ];
        const auto& aEnd = a[ j ];

        for( size_t i = 0; i < b.size(); i++ ) {
            const size_t j = ( i + 1 ) % b.size();
            const auto& bStart = b[ i ];
            const auto& bEnd = b[ j ];
            if( segmentsIntersect( aStart, aEnd, bStart, bEnd, unused ) ) {
                return false;
            }
        }
    }

    return true;
}

bool polygonInsidePolygon( const std::vector< Vector2 >& a, const std::vector< Vector2 >& b )
{
    const Polygons bPolys{ b };
    for( const auto& fromA : a ) {
        if( !insidePolygons( fromA, bPolys ) ) {
            return false;
        }
    }
    return true;
}

bool insidePolygons( const Vector2& point, const std::vector< std::vector< Vector2 > >& polygons )
{
    size_t numHits = 0;
    for( const auto& polygon : polygons ) {
        for( size_t i = 0; i < polygon.size(); i++ ) {
            const Vector2& a = polygon[ i ];
            const Vector2& b = i == polygon.size() - 1 ? polygon[ 0 ] : polygon[ i + 1 ];
            if( hitsSegment( point, a, b ) ) {
                numHits++;
            }
        }
    }
    return numHits % 2 == 1;
}

bool insidePolygon( const Vector2& point, const std::vector< Vector2 >& polygon )
{
    size_t numHits = 0;
    for( size_t i = 0; i < polygon.size(); i++ ) {
        const Vector2& a = polygon[ i ];
        const Vector2& b = i == polygon.size() - 1 ? polygon[ 0 ] : polygon[ i + 1 ];
        if( hitsSegment( point, a, b ) ) {
            numHits++;
        }
    }
    return numHits % 2 == 1;
}

double distanceToNearestPoint( const Vector2& p, const BoundingBoxd& box )
{
    const double pX = p.x();
    const double pY = p.y();

    const Vector2 center = box.center();

    const bool insideX = pX >= box.xMin() && pX <= box.xMax();
    const bool insideY = pY >= box.yMin() && pY <= box.yMax();

    if( insideX && insideY ) {
        return 0.0;
    } else if( insideX ) {
        return std::abs( pY - center.y() ) - box.heightExclusive() / 2.0;
    } else if( insideY ) {
        return std::abs( pX - center.x() ) - box.widthExclusive() / 2.0;
    } else {
        Vector2 corner;
        if( pX < center.x() ) {
            if( pY < center.y() ) {
                corner = box.topLeft();
            } else {
                corner = box.bottomLeft();
            }
        } else {
            if( pY < center.y() ) {
                corner = box.topRight();
            } else {
                corner = box.bottomRight();
            }
        }
        return ( corner - p ).length();
    }
}

double distanceToFarthestPoint( const Vector2& p, const BoundingBoxd& box )
{
    const double pX = p.x();
    const double pY = p.y();
    const Vector2 center = box.center();
    Vector2 corner;
    if( pX < center.x() ) {
        if( pY < center.y() ) {
            corner = box.bottomRight();
        } else {
            corner = box.topRight();
        }
    } else {
        if( pY < center.y() ) {
            corner = box.bottomLeft();
        } else {
            corner = box.topLeft();
        }
    }
    return ( corner - p ).length();
}

bool counterclockwise( const Vector2& startDirection, const Vector2& finishDirection )
{
    // From https://math.stackexchange.com/questions/74307/two-2d-vector-angle-clockwise-predicate
    const double c_z = startDirection.x() * finishDirection.y() - startDirection.y() * finishDirection.x();
    return c_z >= 0.0;
}

bool clockwise( const std::vector< Vector2 >& polygon )
{
    double sum = 0;
    for( size_t i = 0; i < polygon.size(); i++ ) {
        const auto& a = polygon[ i ];
        const auto& b = polygon[ ( i + 1 ) % polygon.size() ];
        sum += ( b.x() - a.x() ) * ( b.y() + a.y() );
    }
    return sum > 0;
}

bool vectorInsideAngle( const Vector2& toTest, const Vector2& a, const Vector2& b )
{
    // https://stackoverflow.com/questions/13640931/how-to-determine-if-a-vector-is-between-two-other-vectors.
    const double aToTest = Vector2::crossProductZ( a, toTest );
    const double bToTest = Vector2::crossProductZ( b, toTest );
    const double aToB = Vector2::crossProductZ( a, b );
    return aToTest * bToTest < 0 && aToTest * aToB > 0;
}

double absAngleChange( const Vector2& a, const Vector2& b, const Vector2& c )
{
    Vector2 first = b - a;
    first.normalize();
    Vector2 second = c - b;
    second.normalize();
    const double dot = Vector2::dot( first, second );
    return safeAcos( dot );
}

double signedAngleBetweenVectors( const Vector2& origin, const Vector2& a, const Vector2& b )
{
    Vector2 first = a - origin;
    first.normalize();
    Vector2 second = b - origin;
    second.normalize();
    const double dot = Vector2::dot( first, second );
    double angle = safeAcos( dot );

    // Change the sign of the angle according to Ledbetter's answer at
    // https://stackoverflow.com/questions/2150050/finding-signed-angle-between-vectors
    Vector2 perpToOne = first;
    perpToOne.turnPerpendicular();
    if( Vector2::dot( perpToOne, second ) < 0 ) {
        angle = -angle;
    }
    return angle;
}

double safeAcos( double cosine )
{
    if( cosine < -1.0 ) {
        return boost::math::constants::pi< double >();
    } else if ( cosine > 1.0 ) {
        return 0.0;
    } else {
        return std::acos( cosine );
    }
}

int combinations(int groupSize, int combinationSize)
{
    return factorial(groupSize)/(factorial(combinationSize)*factorial(groupSize-combinationSize));
}

int factorial(int k)
{
    if( k == 0 ) {
        return 1;
    } else {
        int toReturn = k;
        while(k>2)
        {
            k--;
            toReturn *= k;
        }
        return toReturn;
    }
}

int jumpfloodInitialK(int imageWidth, int imageHeight)
{
    int targetDim = std::max(imageWidth,imageHeight);
    //calculate initial k.
    // exp = logBase2 of (next power of 2 from target Dim) -1
    // k = 2^exp

    int exponent = (int)(ceil(log((double)targetDim)/log(2.0))) - 1;
    return (int)pow(2.0,exponent);

}

double distToLineSegment( const Vector2& p, const Vector2& a, const Vector2& b )
{
    double ret = 0.;
    closestPointOnLineSegment( p, a, b, ret );
    return ret;
}

Vector2 closestPointOnLineSegment( const Vector2& point, const Vector2& a, const Vector2& b, double& outDist )
{    
    const Vector2 proj = projectPointOntoLine( point, a, b );
    // Does 'proj' appear to fall between 'a' and 'b'?
    if( Vector2::dot( proj - a, proj - b ) <= 0 ) {
        outDist = ( proj - point ).length();
        return proj;
    } else {
        const double toA = ( a - point ).length();
        const double toB = ( b - point ).length();
        if( toA < toB ) {
            outDist = toA;
            return a;
        } else {
            outDist = toB;
            return b;
        }
    }
}

double signedDistToLine( const Vector2& point, const LineSegment& l )
{
    return signedDistToLine( point, l.a, l.b );
}

double signedDistToLine( const Vector2& point, const Vector2& a, const Vector2& b )
{
    auto normal = b - a;
    normal.turnPerpendicular();
    normal.normalize();
    return Vector2::dot( normal, point - a );
}

Vector3 projectPointOntoLine( const Vector3& point, const Vector3& lineA, const Vector3& lineB )
{
    auto aToB = lineA - lineB;
    aToB.normalize();
    double u = Vector3::dot( aToB, point - lineA );
    return lineA + aToB * u;
}

Vector2 projectPointOntoLine(const Vector2& point, const Vector2& a, const Vector2& b)
{
    Vector2 aToB = b-a;
    aToB.normalize();
    double u = Vector2::dot(aToB,point-a);
    return a + aToB*u;
}

bool segmentsIntersect( const LineSegment& a, const LineSegment& b, Vector2& store )
{
    return segmentsIntersect( a.a, a.b, b.a, b.b, store );
}

bool segmentsIntersect(const Vector2& a1, const Vector2& a2, const Vector2& b1, const Vector2& b2, Vector2& cross )
{
    Vector3 lineA = lineHomogeneous(a1,a2);
    Vector3 lineB = lineHomogeneous(b1,b2);
    bool valid=false;
    cross = lineIntersection(lineA,lineB,valid);
    if(!valid)
    {
        return false;
    }

    double cX=cross.x();
    double cY=cross.y();

    const double epsilon = 1e-6;

    double xMin = std::min(a1.x(),a2.x()) - epsilon;
    double xMax = std::max(a1.x(),a2.x()) + epsilon;
    double yMin = std::min(a1.y(),a2.y()) - epsilon;
    double yMax = std::max(a1.y(),a2.y()) + epsilon;
    if(cX<xMin || cX>xMax || cY<yMin || cY>yMax)
    {
        return false;
    }
    xMin = std::min(b1.x(),b2.x()) - epsilon;
    xMax = std::max(b1.x(),b2.x()) + epsilon;
    yMin = std::min(b1.y(),b2.y()) - epsilon;
    yMax = std::max(b1.y(),b2.y()) + epsilon;
    if(cX<xMin || cX>xMax || cY<yMin || cY>yMax)
    {
        return false;
    }

    return true;
}

int randInt(int minInclusive, int maxInclusive)
{
    return (rand()%(maxInclusive-minInclusive+1) + minInclusive);
}

Vector3 interpolateOnQuad(const Vector3& topLeft, const Vector3& topRight, const Vector3& bottomRight,
                          const Vector3& bottomLeft, double u, double v)
{
    Vector3 uPoint1 = topLeft + (topRight-topLeft)*u;
    Vector3 uPoint2 = bottomLeft + (bottomRight-bottomLeft)*u;
    return uPoint1 + (uPoint2-uPoint1)*v;
}

Vector2 interpolateOnQuad(const Vector2& topLeft, const Vector2& topRight, const Vector2& bottomRight,
                          const Vector2& bottomLeft, double u, double v)
{
    Vector2 uPoint1 = topLeft + (topRight-topLeft)*u;
    Vector2 uPoint2 = bottomLeft + (bottomRight-bottomLeft)*u;
    return uPoint1 + (uPoint2-uPoint1)*v;
}

Vector2 randNormal2()
{
    Vector2 toReturn( randDouble(-1,1), randDouble(-1,1));
    toReturn.normalize();
    return toReturn;
}

Vector3 randColor()
{
    Vector3 toReturn(randDouble(0,1),randDouble(0,1),randDouble(0,1));
    return toReturn;
}

bool closeEnough(double a, double b, double epsilon)
{
    return fabs(a-b)<epsilon;
}

bool closeEnough( const Vector2& a, const Vector2& b, double epsilon )
{
    return closeEnough( a.x(), b.x(), epsilon ) && closeEnough( a.y(), b.y(), epsilon );
}

bool closeEnough( const Vector3& a, const Vector3& b, double epsilon )
{
    return closeEnough( a.x(), b.x(), epsilon )
        && closeEnough( a.y(), b.y(), epsilon )
        && closeEnough( a.z(), b.z(), epsilon );
}

bool closeEnoughToZero(double a)
{
    return closeEnough(a,0.0);
}

double randDouble( const BoundingIntervald& interval )
{
    return randDouble( interval.min(), interval.max() );
}

double randDouble(double minInclusive, double maxExclusive)
{
    double zeroToOne = (static_cast <double> (rand())) / (static_cast <double> (RAND_MAX));
    return minInclusive + (maxExclusive-minInclusive)*zeroToOne;
}

Vector3 lineHomogeneous( const LineSegment& aAndB )
{
    return lineHomogeneous( aAndB.a, aAndB.b );
}

Vector3 lineHomogeneous(const Vector2& a, const Vector2& b)
{
    return Vector3::cross(Vector3(a.x(),a.y(),1),Vector3(b.x(),b.y(),1));
}

boost::optional< Vector2 > rayLineSegmentIntersection(
    const Vector2& rayOrigin,
    const Vector2& rayNext,
    const Vector2& segA,
    const Vector2& segB )
{
    boost::optional< Vector2 > p = rayLineIntersection( rayOrigin, rayNext, segA, segB );
    if( p.is_initialized() ) {
        // This test is meant to be robust to cases where the line segment is perfectly vertical or
        // perfectly horizontal, such that a simple bounding-box test might fail due to floating-point
        // error.
        return Vector2::dot( p.value() - segA, p.value() - segB ) <= 0 ? p : boost::none;
    } else {
        return p;
    }
}

boost::optional< Vector2 > lineLineSegmentIntersection( const Vector2& lineA, const Vector2& lineB, const Vector2& segA, const Vector2& segB )
{
    bool valid = false;
    const Vector3 lineH = lineHomogeneous( lineA, lineB );
    const Vector3 segH = lineHomogeneous( segA, segB );
    Vector2 lineInt = lineIntersection( lineH, segH, valid );
    if( valid ) {
        if( Vector2::dot( lineInt - segA, lineInt - segB ) <= 0 ) {
            return lineInt;
        } else {
            return boost::none;
        }
    } else {
        return boost::none;
    }
}

boost::optional< Vector2 > rayLineIntersection(
    const Vector2& rayOrigin,
    const Vector2& rayNext,
    const Vector2& lineA,
    const Vector2& lineB )
{
    const Vector3 rayLine = lineHomogeneous( rayOrigin, rayNext );
    const Vector3 line = lineHomogeneous( lineA, lineB );

    bool valid = false;
    const Vector2 cross = lineIntersection( rayLine, line, valid );
    if( !valid )
    {
        return boost::none;
    }

    // Make sure that the intersection lies on the right side of the ray origin.
    if( Vector2::dot( rayNext - rayOrigin, cross - rayOrigin ) >= 0 ) {
        return cross;
    } else {
        return boost::none;
    }
}

Vector2 lineIntersection( const LineSegment& onA, const LineSegment& onB, bool& valid )
{
    return lineIntersection( lineHomogeneous( onA ), lineHomogeneous( onB ), valid );
}

Vector2 lineIntersection(const Vector3& lineHomogeneousA, const Vector3& lineHomogeneousB, bool& valid)
{
    valid=true;
    Vector3 intersection = Vector3::cross(lineHomogeneousA,lineHomogeneousB);
    if(closeEnoughToZero(intersection.z()))
    {
        valid=false;
    }
    return Vector2(intersection.x()/intersection.z(),intersection.y()/intersection.z());
}

// This code adapted from http://wiki.beyondunreal.com/HSV-RGB_Conversion
// rgb.x(),rgb.y(),rgb.z() in [0,1]
// return { [0,360], [0,1], [0,1] }
Vector3 rgbToHsv(const Vector3& rgb)
{

    double max=0,min=0,chroma=0;

    min = std::min(std::min(rgb.r(), rgb.g()), rgb.b());
    max = std::max(std::max(rgb.r(), rgb.g()), rgb.b());
    chroma = max-min;

    double hue=0,saturation=0,value=0;

    //If chroma is 0, then S is 0 by definition, and H is undefined but 0 by convention.
    if(!closeEnoughToZero(chroma))
    {
        if(closeEnough(rgb.r(),max))
        {
            hue = (rgb.g() - rgb.b()) / chroma;

            if(hue < 0.0)
            {
                hue += 6.0;
            }
        }
        else if(closeEnough(rgb.g(),max))
        {
            hue = ((rgb.b() - rgb.r()) / chroma) + 2.0;
        }
        else
        {
            hue = ((rgb.r() - rgb.g()) / chroma) + 4.0;
        }

        hue *= 60.0;
        saturation = chroma / max;
    }

    value = max;

    return Vector3(hue,saturation,value);
}

//Code adapted from http://en.wikipedia.org/wiki/HSL_and_HSV#Converting_to_RGB
//takes in { [0,360], [0,1], [0,1] }
Vector3 hsvToRgb(const Vector3& hsv)
{
    double hue = hsv.x();
    double saturation = hsv.y();
    double value = hsv.z();
    double chroma=value*saturation;
    double huePrime = hue/60.0;
    double x=  chroma*(1.0 - fabs( fmod(huePrime,2.0) -1.0  ));

    double r=0,g=0,b=0;

    if(huePrime>=0 && huePrime<1)
    {
        r=chroma;
        g=x;
        b=0;
    }
    if(huePrime>=1 && huePrime<2)
    {
        r=x;
        g=chroma;
        b=0;
    }
    if(huePrime>=2 && huePrime<3)
    {
        r=0;
        g=chroma;
        b=x;
    }
    if(huePrime>=3 && huePrime<4)
    {
        r=0;
        g=x;
        b=chroma;
    }
    if(huePrime>=4 && huePrime<5)
    {
        r=x;
        g=0;
        b=chroma;
    }
    if(huePrime>=5 && huePrime<6)
    {
        r=chroma;
        g=0;
        b=x;
    }
    double m =value-chroma;
    r+=m;
    g+=m;
    b+=m;
    return Vector3(r,g,b);
}

Vector3 rgbToCielab(const Vector3& rgb)
{
    Vector3 cielab = rgbToXyz(rgb);
    return xyzToCielab(cielab);
}

Vector3 cielabToRgb(const Vector3& cielab)
{
    Vector3 rgb = cielabToXyz(cielab);
    return xyzToRgb(rgb);
}

//the stimulus values are sRGB-based (D65 white point) and come from the table at http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
//method comes from http://en.wikipedia.org/wiki/SRGB#cite_note-orig_pub-2
Vector3 rgbToXyz(const Vector3& rgb)
{
    const double cutoff = 0.04045;
    const double divisor = 12.92;
    const double alpha  =0.055;
    const double powVal=2.4;

    double r=rgb.r();
    double g=rgb.g();
    double b=rgb.b();

    double rLinear = r <=cutoff ? r/divisor : pow((r+alpha)/(1.0+alpha),powVal);
    double gLinear = g <=cutoff ? g/divisor : pow((g+alpha)/(1.0+alpha),powVal);
    double bLinear = b <=cutoff ? b/divisor : pow((b+alpha)/(1.0+alpha),powVal);

    return Vector3(0.4124564*rLinear + 0.3575761*gLinear + 0.1804375*bLinear,
                   0.2126729*rLinear + 0.7151522*gLinear + 0.0721750*bLinear,
                   0.0193339*rLinear + 0.1191920*gLinear + 0.9503041*bLinear);
}

//the stimulus values are sRGB-based (D65 white point) and come from the table at http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
//method comes from http://en.wikipedia.org/wiki/SRGB#cite_note-orig_pub-2
Vector3 xyzToRgb(const Vector3& xyz)
{
    double x=xyz.x();
    double y=xyz.y();
    double z=xyz.z();

    double rLinear = 3.2404542*x -1.5371385*y -0.4985314*z;
    double gLinear = -0.9692660*x + 1.8760108*y +0.0415560*z;
    double bLinear = 0.0556434*x - 0.2040259*y + 1.0572252*z;

    const double cutoff = 0.0031308;
    const double factor = 12.92;
    const double alpha = 0.055;
    const double powVal = 1.0/2.4;
    return Vector3(
                rLinear <= cutoff ? factor*rLinear : (1.0+alpha)*pow(rLinear,powVal)-alpha,
                gLinear <= cutoff ? factor*gLinear : (1.0+alpha)*pow(gLinear,powVal)-alpha,
                bLinear <= cutoff ? factor*bLinear : (1.0+alpha)*pow(bLinear,powVal)-alpha
                );
}

//taken from http://www.cs.rit.edu/~ncs/color/t_convert.html#XYZ%20to%20CIE%20L*a*b*%20%28CIELAB%29%20&%20CIELAB%20to%20XYZ
Vector3 xyzToCielab(const Vector3& xyz)
{
    double x=xyz.x();
    double y=xyz.y();
    double z=xyz.z();
    x/=whitePointX;
    y/=whitePointY;
    z/=whitePointZ;

    const double cutoff = 0.008856;
    const double cubeRoot = 1.0/3.0;
    const double addTerm = 16.0/116.0;
    const double multTerm=7.787;

    double fX = x>cutoff ? pow(x,cubeRoot) : multTerm*x + addTerm;
    double fY = y>cutoff ? pow(y,cubeRoot) : multTerm*y + addTerm;
    double fZ = z>cutoff ? pow(z,cubeRoot) : multTerm*z + addTerm;

    return Vector3(
                y>cutoff ? 116.0*pow(y,cubeRoot) - 16.0 : 903.3*y,
                500.0*(fX-fY),
                200.0*(fY-fZ)
                );
}

//from http://www.cs.rit.edu/~ncs/color/t_convert.html#XYZ%20to%20CIE%20L*a*b*%20%28CIELAB%29%20&%20CIELAB%20to%20XYZ
//had to supplement with http://en.wikipedia.org/wiki/Lab_color_space#Forward_transformation
//because the instructions in the first link do not properly account for the functions' discontinuities
Vector3 cielabToXyz(const Vector3& lab)
{
    double L=lab.x();
    double a=lab.y();
    double b=lab.z();

    double fx=(1.0/116.0)*(L+16.0)+(1.0/500.0)*a;
    double fz=(1.0/116.0)*(L+16.0)-(1.0/200.0)*b;
    double fy=(1.0/116.0)*(L+16.0);

    const double cutoff=6.0/29.0;
    const double factor=pow(cutoff,2.0)*3.0;
    const double term = 4.0/29.0;
    return Vector3(
                whitePointX*( fx>cutoff ? pow(fx,3.0) : factor*(fx-term)),
                whitePointY*( fy>cutoff ? pow(fy,3.0) : factor*(fy-term)),
                whitePointZ*( fz>cutoff ? pow(fz,3.0) : factor*(fz-term))
                );
}

double distToPolygon( const Vector2& p, const std::vector< Vector2 >& polyline )
{
    if( polyline.size() > 1 ) {
        double smallest = std::numeric_limits< double >::max();
        for( size_t i = 0; i < polyline.size(); i++ ) {
            const auto& a = polyline[ i ];
            const auto& b = polyline[ i == polyline.size() - 1 ? 0 : i + 1 ];
            double dist = 0.0;
            mathUtility::closestPointOnLineSegment( p, a, b, dist );
            if( dist < smallest ) {
                smallest = dist;
            }
        }
        return smallest;
    } else {
        return ( p - polyline.front() ).length();
    }
}

double distBetweenPolylines( const std::vector< Vector2 >& a, const std::vector< Vector2 >& b )
{
    Vector2 unused;
    double shortest = std::numeric_limits< double >::max();
    for( const auto& fromA : a ) {
        const auto dist = distToPolyline( fromA, b, unused );
        if( dist < shortest ) {
            shortest = dist;
        }
    }
    for( const auto& fromB : b ) {
        const auto dist = distToPolyline( fromB, a, unused );
        if( dist < shortest ) {
            shortest = dist;
        }
    }

    for( int i = 0; i < static_cast< int >( a.size() - 1 ); i++ ) {
        for( int j = 0; j < static_cast< int >( b.size() - 1 ); j++ ) {
            if( segmentsIntersect( a[ i ], a[ i + 1 ], b[ j ], b[ j + 1 ], unused ) ) {
                return 0.0;
            }
        }
    }
    return shortest;
}

double distToPolyline( const Vector2& p, const std::vector< Vector2 >& polyline, Vector2& storeClosestPoint )
{
    if( polyline.size() > 1 ) {
        double smallest = std::numeric_limits< double >::max();
        for( size_t i = 0; i < polyline.size() - 1; i++ ) {
            const auto& a = polyline[ i ];
            const auto& b = polyline[ i + 1 ];
            double dist = 0.0;
            const auto closestPoint = mathUtility::closestPointOnLineSegment( p, a, b, dist );
            if( dist < smallest ) {
                storeClosestPoint = closestPoint;
                smallest = dist;
            }
        }
        return smallest;
    } else {
        storeClosestPoint = polyline.front();
        return ( p - polyline.front() ).length();
    }
}

std::array< int, 3 > rgbFloatToInt( const Vector3& rgb )
{
    return
    {
        static_cast< int >( rgb.x() * 255.0 ),
        static_cast< int >( rgb.y() * 255.0 ),
        static_cast< int >( rgb.z() * 255.0 )
    };
}

Vector3 rgbIntFoFloat( const std::array< int, 3 >& rgb )
{
    return Vector3(
        static_cast< double >( rgb[ 0 ] ) / 255.0,
        static_cast< double >( rgb[ 1 ] ) / 255.0,
        static_cast< double >( rgb[ 2 ] ) / 255.0
    );
}

double polylineLength( const std::vector< Vector2 >& p )
{
    double total = 0;
    for( size_t i = 1; i < p.size(); i++ ) {
        total += ( p[ i ] - p[ i - 1 ] ).length();
    }
    return total;
}

std::vector< Vector2 > lineSegmentCircleIntersection( const Vector2& segA, const Vector2& segB, const Vector2& center, double rad )
{
    const auto lineRes = lineCircleIntersection( segA, segB, center, rad );
    if( lineRes.size() ) {
        std::vector< Vector2 > ret;
        for( const auto& p : lineRes ) {
            // Is it on the seg
            if( Vector2::dot( ( p - segA ), ( p - segB ) ) <= 0. ) {
                ret.push_back( p );
            }
        }
        return ret;
    } else {
        return {};
    }
}

std::vector< Vector2 > lineCircleIntersection( const LineSegment& line, const Vector2& center, double rad )
{
    return lineCircleIntersection( line.a, line.b, center, rad );
}

std::vector< Vector2 > lineCircleIntersection( const Vector2& lineA, const Vector2& lineB, const Vector2& center, double r )
{
    // From https://mathworld.wolfram.com/Circle-LineIntersection.html

    const auto x_1 = lineA.x() - center.x();
    const auto y_1 = lineA.y() - center.y();
    const auto x_2 = lineB.x() - center.x();
    const auto y_2 = lineB.y() - center.y();

    const auto d_x = x_2 - x_1;
    const auto d_y = y_2 - y_1;
    const auto d_r2 = d_x * d_x + d_y * d_y;
    const auto D = x_1 * y_2 - x_2 * y_1;

    const double sgn = d_y < 0 ? -1. : 1.;

    const auto toRoot = r * r * d_r2 - D * D;
    if( toRoot < 0 ) {
        return {};
    }
    const auto root = std::sqrt( toRoot );

    const size_t numRoots = root == 0. ? 1 : 2;
    std::vector< Vector2 > ret;
    for( size_t i = 0; i < numRoots; i++ ) {
        const double plusOrMinus = i == 0 ? 1. : -1.;
        const double x = ( D * d_y + plusOrMinus * sgn * d_x * root ) / d_r2;
        const double y = ( -D * d_x + plusOrMinus * std::abs( d_y ) * root ) / d_r2;
        ret.push_back( Vector2( x + center.x(), y + center.y() ) );
    }
    return ret;
}

} // mathUtility
} // core
