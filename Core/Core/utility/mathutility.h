#ifndef CORE_MATHUTILITY_H
#define CORE_MATHUTILITY_H

#include <Core/utility/boundingbox.h>

#include <boost/optional.hpp>
#include <boost/math/constants/constants.hpp>

#include <vector>

/// Convert the 'i' in the canonical "for( i = 0; i < iMaxExclusive; i++)" to a number from 0 to 1 (including the 1).
/// Both numbers are expected to be integral and greater than -1. 'i' must be less than 'iMaxExclusive'.
/// (Ternary op is from old fear that n / n won't always return precisely 1.)
#define F_FROM_I( i, iMaxExclusive ) ( ( i + 1 ) == iMaxExclusive ? 1.0 : static_cast< double >( i ) / static_cast< double >( iMaxExclusive - 1 ) )

#define CORE_PI (boost::math::constants::pi< double >())

namespace core {

struct LineSegment;
class Vector2;
class Vector3;

namespace mathUtility {

template< typename T >
constexpr T degreesToRadians( T degrees )
{
    return degrees / 360.0 * 2.0 * boost::math::constants::pi< T >();
}

template< typename T >
constexpr T radiansToDegrees( T radians )
{
    return radians / ( 2.0 * boost::math::constants::pi< T >() ) * 360.0;
}

std::vector< Vector2 > lineCircleIntersection( const LineSegment& line, const Vector2& center, double rad );
std::vector< Vector2 > lineCircleIntersection( const Vector2& lineA, const Vector2& lineB, const Vector2& center, double rad );
std::vector< Vector2 > lineSegmentCircleIntersection( const Vector2& segA, const Vector2& segB, const Vector2& center, double rad );

/// 'planeNormal' need not be normalized
Vector3 rayIntersectPlane( const Vector3& rayA, const Vector3& rayB, const Vector3& onPlane, const Vector3& planeNormal, bool& valid );
/// 'planeNormal' must be normalized.
Vector3 projectPointToPlane( const Vector3& point, const Vector3& onPlane, const Vector3& planeNormal );
Vector3 projectPointOntoLine( const Vector3& point, const Vector3& lineA, const Vector3& lineB );

/// Let 'a1' and 'a2' represent line A and 'b1' and 'b2' line B. Return the point on line B closest to line A.
Vector3 nearestPointOnOtherLine( const Vector3& a1, const Vector3& a2, const Vector3& b1, const Vector3& b2 );
/// Return the point on the segment identified by 'segA' and 'segB' to the line identified by 'lineA' and 'lineB'.
Vector3 nearestPointOnSegmentToLine( const Vector3& segA, const Vector3& segB, const Vector3& lineA, const Vector3& lineB );

/// Return whether 'point' is inside 'polygons', a group of implicitly closed polylines (no need to repeat starting point at end).
/// 'polygons' might represent two separate polygons or one main polygon and holes
/// inside it (and holes inside holes inside holes...).
bool insidePolygons( const Vector2& point, const std::vector< std::vector< Vector2 > >& polygons );
bool insidePolygon( const Vector2& point, const std::vector< Vector2 >& polygon );
/// Consider 'polyline' to be open (not implicitly closed, at any rate).
double distToPolyline( const Vector2& p, const std::vector< Vector2 >& polyline, Vector2& storeClosestPoint );
/// Consider 'polygon' to be closed.
double distToPolygon( const Vector2& p, const std::vector< Vector2 >& polygon );
/// Return true if 'a' and 'b' have no intersection, 'a' is not completely inside 'b', and 'b' is not completely inside 'a'.
/// 'a' and 'b' are considered closed (they do not need to repeat their starting vertices).
bool polygonsDoNotIntersect( const std::vector< Vector2 >& a, const std::vector< Vector2 >& b );
/// Return whether 'a' is completely inside 'b'. Both polygons are considered closed.
bool polygonInsidePolygon( const std::vector< Vector2 >& a, const std::vector< Vector2 >& b );
/// If the two polylines intersect, return 0.0.
double distBetweenPolylines( const std::vector< Vector2 >&, const std::vector< Vector2 >& );
/// Return the length of the open polyline.
double polylineLength( const std::vector< Vector2 >& );

/// Take a triplet 'rgb' in [0,1] and return a triplet in [0,255].
std::array< int, 3 > rgbFloatToInt( const Vector3& rgb );
Vector3 rgbIntFoFloat( const std::array< int, 3 >& );

/// Return a line segment representing the part of 'line' inside 'box', or boost::none of there is
/// none. Warning: I think there are edge cases (like where 'line' exactly matches one of the box's
/// diagonals) where this will erroneously return boost::none.
boost::optional< LineSegment > keepLineInBox( const LineSegment& line, const BoundingBoxd& box );
boost::optional< LineSegment > keepLineInBox( const LineSegment& line, const Vector2& boxCornerA, const Vector2& boxCornerB );
boost::optional< LineSegment > keepRayInBox( const Vector2& rayA, const Vector2& rayB, const BoundingBoxd& );

//from http://en.wikipedia.org/wiki/Illuminant_D65 (I realized that
//I needed to divide those values by 100.0 when I started comparing my results to
//what I was getting from http://davengrace.com/cgi-bin/cspace.pl
static const double whitePointX = 0.95047;
static const double whitePointY = 1.0;
static const double whitePointZ = 1.08883;

double smoothstep( const double edge0, const double edge1, const double x );

template< typename T >
T lerp( const T& a, const T& b, const double& f )
{
    return static_cast< T >( a * ( 1.0 - f ) + b * f );
}

template< typename T >
T clamp( const T& toClamp, const T& minInclusive, const T& maxInclusive )
{
    return std::min< T >( maxInclusive, std::max< T >( toClamp, minInclusive ) );
}

/// Return the distance between 'p' and the nearest point on the box outline, or 0 if
/// 'p' is inside the box.
double distanceToNearestPoint( const Vector2& p, const BoundingBoxd& box );
/// Return the distance between 'p' and the point on the box outline farthest from 'p'.
double distanceToFarthestPoint( const Vector2& p, const BoundingBoxd& box );

/// Clamp 'cosine' to [-1,1] and return its acos.
double safeAcos( double cosine );

int combinations(int groupSize, int combinationSize);
int factorial(int k);
int randInt(int minInclusive, int maxInclusive);
double randDouble(double minInclusive, double maxExclusive);
double randDouble( const BoundingIntervald& );
bool closeEnough( double a, double b, double epsilon = 1e-6 );
bool closeEnough( const Vector2&, const Vector2&, double epsilon = 1e-6 );
bool closeEnough( const Vector3&, const Vector3&, double epsilon = 1e-6 );
bool closeEnoughToZero(double a);

int jumpfloodInitialK(int imageWidth, int imageHeight);

Vector2 randNormal2();
Vector3 randColor();

/// Return whether 'toTest' directionally lies within the angle from 'a' to 'b'. All inputs must be nonzero.
/// If 'a' and 'b' are parallel, return false. If 'toTest' is parallel with either 'a' or 'b', return false.
bool vectorInsideAngle( const Vector2& toTest, const Vector2& a, const Vector2& b );
bool counterclockwise( const Vector2& startDirection, const Vector2& finishDirection );
/// Return whether the polygon (implicitly closed; doesn't need to repeat the starting position) has counterclockwise
/// winding order.
bool clockwise( const std::vector< Vector2 >& polygon );

/// Return the angle between origin->a and origin->b in radians.
double signedAngleBetweenVectors( const Vector2& origin, const Vector2& a, const Vector2& b );
/// Return the absolute value of the angle change between a->b and b->c.
double absAngleChange( const Vector2& a, const Vector2& b, const Vector2& c );

Vector3 lineHomogeneous( const Vector2& a, const Vector2& b );
Vector3 lineHomogeneous( const LineSegment& aAndB );
Vector2 lineIntersection( const Vector3& lineHomogeneousA, const Vector3& lineHomogeneousB, bool& valid );
Vector2 lineIntersection( const LineSegment& onA, const LineSegment& onB, bool& valid );

/// Return the intersection between the ray starting at 'rayOrigin' and passing through 'rayNext' and the line passing through
/// 'lineA' and 'lineB'.
boost::optional< Vector2 > rayLineIntersection( const Vector2& rayOrigin, const Vector2& rayNext, const Vector2& lineA, const Vector2& lineB );
/// Return the intersection between the ray starting at 'rayOrigin' and passing through 'rayNext' and the line segment from 'segA' to 'segB'.
boost::optional< Vector2 > rayLineSegmentIntersection( const Vector2& rayOrigin, const Vector2& rayNext, const Vector2& segA, const Vector2& segB );
boost::optional< Vector2 > lineLineSegmentIntersection( const Vector2& lineA, const Vector2& lineB, const Vector2& segA, const Vector2& segB );

/// rotations are counterclockwise if 'radians' positive
Vector2 rotate( const Vector2& toRotate, const Vector2& about, double radians );
Vector2 rotate( const Vector2& toRotate, const Vector2& about, double cosRadians, double sinRadians );

double signedDistToLine( const Vector2& point, const Vector2& a, const Vector2& b );
double signedDistToLine( const Vector2& point, const LineSegment& );
Vector2 projectPointOntoLine(const Vector2& point, const Vector2& a, const Vector2& b);
bool segmentsIntersect( const Vector2& a1, const Vector2& a2, const Vector2& b1, const Vector2& b2, Vector2& storeIntersection );
bool segmentsIntersect( const LineSegment& a, const LineSegment& b, Vector2& storeIntersection );
Vector2 closestPointOnLineSegment( const Vector2& point, const Vector2& a, const Vector2& b, double& outDist );
double distToLineSegment( const Vector2& p, const Vector2& a, const Vector2& b );

Vector3 interpolateOnQuad(const Vector3& topLeft, const Vector3& topRight, const Vector3& bottomRight,
                          const Vector3& bottomLeft, double u, double v);
Vector2 interpolateOnQuad(const Vector2& topLeft, const Vector2& topRight, const Vector2& bottomRight,
                         const Vector2& bottomLeft, double u, double v);

Vector3 hsvToRgb(const Vector3& hsv);
Vector3 rgbToHsv(const Vector3& rgb);
//this means CIE XYZ - the conversion assumes RGB means non-linear sRGB w/ D65 white point
Vector3 rgbToXyz(const Vector3& rgb);
//assumes that RGB means non-linear sRGB w/ D65 white point
Vector3 xyzToRgb(const Vector3& xyz);
//output channels are _not_ constrained to [0,1]
Vector3 xyzToCielab(const Vector3& xyz);
//output channels are _not_ constrained to [0,1]
Vector3 cielabToXyz(const Vector3& lab);
Vector3 rgbToCielab(const Vector3& rgb);
//output channels are _not_ constrained to [0,1]
Vector3 cielabToRgb(const Vector3& cielab);

} // mathUtility
} // core

#endif // #include guard
