#ifndef CORE_BSPLINE2UTILITY_H
#define CORE_BSPLINE2UTILITY_H

#include <Core/utility/boundingbox.h>
#include <Core/utility/curvecurveintersection.h>
#include <Core/utility/intersectionparameters.h>

#include <memory>

namespace coreTest
{
class BSplineTests;
} // coreTest

namespace core {

class BSpline2;
class Vector2;
struct LineSegment;

class BSpline2Utility
{
public:
    using Spline = BSpline2;
    using UniqueCurve = std::unique_ptr< Spline >;

    /// Store intersections between 'a' and 'b' in 'store'. Calculate intersections via the subdivision algorithm.
    /// Intersections satisfy the characteristics described in 'IntersectionParameters'; also, two intersections' bounding boxes
    /// must not intersect.
    static void intersections(
        const Spline& a,
        const Spline& b,
        CurveCurveIntersections& store,
        const IntersectionParameters& );

    /// Store the self intersections of 'spline' in 'intersections'.  In each intersection, set 'tIntervalA' to be the earlier T time (interval).
    /// This cannot currently be relied on to detect multiple self-intersections in the same location (see the NBC flower curve).
    /// If 'spline' has cusps whose size is on the order of a box with dimensions no larger than 'minBoxDim', this cusp's intersection
    /// may not register.
    static void selfIntersections(
        const Spline& spline,
        CurveCurveIntersections& intersections,
        const IntersectionParameters& );

    /// Return the t value corresponding to the point on non-null 'spline' nearest 'p'. Iterate until determining that
    /// 'p' is at least a and no more than b away from 'spline' such that b-a<='maxDistInterval'. 'maxDistInterval'
    /// must be greater than 0, and the smaller it is, the longer and more memory intensive this operation will be.
    /// The true distance from 'p' to 'spline' and the (indirectly) returned distance can differ by no more than
    /// 'maxDistInterval'.
    static double nearestPoint( const Vector2& p, const Spline& spline, double maxDistInterval );

    static void lineSegmentIntersections(
        const Spline& a,
        const Vector2& bStart,
        const Vector2& bEnd,
        CurveCurveIntersections& intersections,
        const IntersectionParameters& );
    static void lineSegmentIntersections(
        const Spline& a,
        const LineSegment& b,
        CurveCurveIntersections& intersections,
        const IntersectionParameters& );

    /// Return a spline which is t (in [0,1]) of the way from 'a' to 'b'.
    static UniqueCurve interpolateBetweenSplines( const Spline& a, const Spline& b, double t );

    /// Create a spline which is the sum of 'terms'. Assume that they need to be degree-elevated and knot-supplemented to
    /// be compatible with each other.
    static UniqueCurve addSplines( const std::vector< const Spline* >& terms );

    /// Give 'a' all the knots that 'b' has, and vice versa. Both splines must have the same degree.
    /// Throws 'BuildSplineException'.
    static void unionKnotVectors( Spline& a, Spline& b );

    /// Assuming 'parts' is a C0 chain, return a stitched composite. The resulting spline will have multiple knots and likely be
    /// no better than C0 at each inter-part juncture (unless the parts are already better than C0 with each other).
    ///
    /// 'tWeights' indicates how much of the [0,1] T interval each part gets in the composite curve: if 'tWeights' is [ 1, 2 ],
    /// then the resulting T intervals for the two parts would be [0,1/3] and [2/3,1]. 'tWeights' must have the same size
    /// as 'parts'. All weights must be greater than zero.
    ///
    /// If 'forceClosedShape' is true, force the last control point to equal the first.
    ///
    /// If 'storePartEndT' is non-null, store in it the T value where each part from 'parts' ends in the composite result.
    static UniqueCurve stitchC0Spline(
        const std::vector< const Spline* >& parts,
        const std::vector< double >& tWeights,
        bool forceClosedShape = false,
        std::vector< double >* storePartEndT = nullptr );

    /// A wrapper around 'stitchC0Spline' that uses the parts' lengths as 'tWeights', where lengths are based on
    /// 'lengthPrecision'.
    static UniqueCurve stitchC0Spline(
        const std::vector< const Spline* >& parts,
        const size_t lengthPrecision,
        bool forceClosedShape = false,
        std::vector< double >* storePartEndT = nullptr );

    /// Add 'knotValue' in front of the current i'th knot in 'knots'. If there are already copies of this knot in 'knots', then
    /// 'i' must index the first of them.
    static void insertKnot(
        int i, double knotValue, int numExistingCopies, int degree, std::vector< double >& knots, std::vector< Vector2 >& control );

    /// Get the control point associated with adding knot value 'knot' at index 'i' in 'knots'. For instance,
    /// if the knot vector is currently [ 0 0 0 1 2 2 2 ], 'i' is 3, and 'knot' is 0.5, this will return
    /// the point with knot vector [ 0.5 1 2 ].
    static Vector2 interpFromControlPoints(
        int i, double knot, int degree, const std::vector< Vector2 >& control, const std::vector< double >& knots );
private:
    friend class coreTest::BSplineTests;

    /// Assume that a group of curves are about to be stitched together and together occupy the T range [0,1].
    /// Each weight indicates the relative amount of that interval that the corresponding curve will get.
    /// Return the ending T value for each curve.
    static std::vector< double > tEndValues( const std::vector< double >& componentWeights );
};

} // core

#endif // #include
