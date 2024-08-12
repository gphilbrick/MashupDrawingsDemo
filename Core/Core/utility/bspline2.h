#ifndef CORE_BSPLINE2_H
#define CORE_BSPLINE2_H

#include <Core/utility/boundingbox.h>
#include <Core/utility/curvefitparametrizetype.h>
#include <Core/utility/vector2.h>

#include <array>
#include <functional>
#include <memory>
#include <vector>

namespace gte {
template < int32_t N, typename Real >
class BSplineCurve;
} // gte

namespace coreTest {
class BSplineTests;
} // coreTest

namespace core {

class CurveInterval;
struct LineSegment;

/// A Bezier spline with equal weights (non-rational) and Bezier end conditions ("clamped"). The spline covers the
/// T-interval from 0 to 1. The spline uses "Sederberg knot format," which means a knot vector with 'degree' end-knots at each end.
/// For instance, a degree-2 spline with three control points has knot vector [ 0, 0, 1, 1 ].
/// This object is guaranteed to be valid: it has a degree>=1 and a valid number of control points.
class BSpline2
{
public:
    friend class coreTest::BSplineTests;

    using Type = BSpline2;
    using UniquePtr = std::unique_ptr< Type >;
    using ConstUniquePtr = std::unique_ptr< const Type >;
    using Control = std::vector< Vector2 >;

    BSpline2( const Type& other );
    BSpline2( Type&& other );
    /// Throw 'BuildSplineException' if params invalid.
    BSpline2( int degree, const Control& controlPoints );
    /// Make a nonuniform, open spline with Bezier end conditions. 'intermediateKnots' contains all except the end-knots. For instance, if
    /// the goal is to create a two-curve, degree-3 spline with knot vector [ 0, 0, 0, 0.75, 1, 1, 1 ] (following Sederberg's knot format), then
    /// 'intermediateKnots' should be [ 0.75 ]. 'intermediateKnots' must contain controlPoints.size() - degree - 1 values, which must be increasing.
    /// If there are no internal knots (there is only one Bezier curve), this has the same effect as
    /// createFromControlPoints.
    ///
    /// If 'intermediateKnots' begins with values equal (close enough) to 0 or ends with values equal (close enough) to 1, these will be chopped off.
    /// The underlying WildMagic [now GTE--does it still have this issue?] implementation seems to have problems with these knots, and they don't
    /// produce functionally distinct curves in any case.
    ///
    /// Throw 'BuildSplineException' if params invalid.
    BSpline2( int degree, const Control& controlPoints, const std::vector< double >& intermediateKnots );
    BSpline2& operator = ( const Type& other);
    BSpline2& operator = ( Type&& other);
    virtual ~BSpline2();

    /// Throws 'BuildSplineException'
    void buildFromControlPoints( int degree, const Control& controlPoints );
    /// Throws 'BuildSplineException'
    void buildFromControlPointsAndKnots(
            int degree, const Control& controlPoints, const std::vector< double >& intermediateKnots );

    /// The spline must be valid, and the provided 'degree' must be higher than the current degree.
    void degreeElevate( int degree );
    void reverse();
    void scale( const Vector2& scaleBy );
    void transform( std::function< Vector2( const Vector2& ) > f );

    int degree() const;

    // ltwarning: This interface implies a lot of wasted computation since this class wraps 'gte::BSplineCurve', which is designed
    // to calculate derivative and position at the same time. I don't want to refactor all of my code to work this way though,
    // so for now I'll leave the waste in place.
    /// 't' must be in [0,1].
    Vector2 position( double t ) const;
    /// 't' must be in [0,1].
    Vector2 derivative( double t ) const;
    /// 't' must be in [0,1].
    Vector2 secondDerivative( double t ) const;

    const Control& controlPoints() const;
    /// Return one of the endpoint curvature magnitudes. If there are duplicate control points at either end of the spline,
    /// the return value will be infinity.
    double curvatureMagnitude( bool startOrEnd ) const;
    double curvatureSigned( double t ) const;

    /// Return the curve's length based on at least the precision of using 'precision' points
    /// along the curve. Generate a new cached length if such does not exist yet. 'precision' must be > 1.
    double cachedLength( size_t precision = defaultLengthPrecision ) const;

    std::vector< double > internalKnots() const;
    /// Return the full Sederberg-form knot vector (only 'degree' end knots on each end).
    std::vector< double > fullKnots() const;
    /// Return knots in Greg-form: [0, 1] would be a single Bezier.
    std::vector< double > fullKnotsNoMultiples() const;

    /// Because of Bezier end conditions, these are faster than calling position().
    const Vector2& startPosition() const;
    const Vector2& endPosition() const;
    const Vector2& endpoint( bool startOrEnd ) const;
    /// Return whether the start/end positions are identical (says nothing more about whether the spline is treatable as truly "closed").
    bool endpointsEqual() const;

    /// Construct and return the bounding box.
    BoundingBoxd boundingBox() const;

    /// Split the spline into two splines at the cutoff 't' value. The spline must be valid.
    std::array< UniquePtr, 2 > subdivide( double t ) const;

    /// Return the subcurve specified by 'i'. If 'i' is only applicable to a closed curve, then this curve's endpoints must be equal.
    UniquePtr extractCurveForTInterval( const CurveInterval& i ) const;
    UniquePtr extractCurveForTInterval( double tStart, double tEnd ) const;
    UniquePtr extractCurveForTInterval( const std::array< double, 2 >& interval ) const;
    UniquePtr clone() const;
    UniquePtr reverseCopy() const;
    /// Create a copy of this curve with its control points altered to start and end with 'start' and 'end'.
    UniquePtr c0Copy( const Vector2& start, const Vector2& end ) const;
    /// Create an offset curve 'offset' away to the left or right, depending on 'whichDirection'.
    UniquePtr offset( bool whichDirection, double offset ) const;

    /// Create a polyline approximation using 'numPoints' >=2 as a _guide_, only. If the degree is 1, ignore 'numPoints' and
    /// return a copy of the control points, unless 'ignoreNumPointsIfDeg1' is false.
    std::vector< Vector2 > polylineApproximation( int numPoints, bool ignoreNumPointsIfDeg1 = true ) const;
    /// Crudely generate a polyline by evenly dividing the [0,1] T-interval.
    std::vector< Vector2 > crudePolylineApproximation( size_t numPoints ) const;

    /// Return the control points of the constituent Bezier curves, except for the degenerate ones caused by multiple knots.
    /// Store in 'storeTStarts' the T start value (in the original spline) of each returned Bezier curve.
    std::vector< Control > breakIntoBCurves( std::vector< double >& storeTStarts ) const;

    /// Return T values for approximating 'interval' on 'this' with a polyline, using 'defaultNumPoints' as a loose
    /// guide and attempting to include the T values for corners. Assume degree > 1 or that if this is degree 1, internal points
    /// on the edges between control points are acceptable. The total number of points may be well above 'defaultNumPoints'
    /// in some situations, such as when this curve is a long degree-1 and 'defaultNumPoints' is small.
    std::vector< double > tForPolylineApprox( const std::array< double, 2 >& interval, size_t defaultNumPoints ) const;

    /// Return T values for approximating the entire curve, using 'defaultNumPoints' as a guide but not exceeding
    /// 'maxPoints' in size. 'maxPoints' must be at least 2.
    std::vector< double > tForPolylineApproxLimited( size_t defaultNumPoints, size_t maxPoints ) const;

    /// Create a copy where the last control point is set exactly equal to the first control point.
    UniquePtr forceClosed() const;

    /// Return the number of constituent Bezier curves.
    int numBezierCurves( bool includeDegenerate ) const;

    /// Make a uniform, open spline with Bezier end conditions.
    /// Throw 'BuildSplineException' if params invalid.
    static UniquePtr createFromControlPoints( int degree, const Control& controlPoints );
    /// Throw 'BuildSplineException' if params invalid (see corresponding constructor).
    static UniquePtr createFromControlPointsAndKnots(
        int degree,
        const Control& controlPoints,
        const std::vector< double >& intermediateKnots );

    /// Make a uniform, open spline with Bezier end conditions that
    /// as closely as possible approximates the data points given.
    /// The number of data points must be >= 'numControlPoints' + 'degree' + 1.
    /// Throws 'BuildSplineException'.
    static UniquePtr createFitToDataPoints(
        int degree,
        int numControlPoints,
        const Control& dataPoints,
        CurveFitParametrizeType parametrize = ChordLength );
    /// If there are enough samples in 'dataPoints', fit a cubic spline with 'numCurves' curves. Otherwise, use reasonable
    /// fallbacks (lower degree, lower number of control points), etc., favoring a smooth curve (degree>1) as much as possible.
    /// If 'dataPoints' has size 0, do nothing.
    /// Throws 'BuildSplineException'.
    static UniquePtr createFitToDataOrFallback(
        int numCurves,
        const Control& dataPoints,
        CurveFitParametrizeType parametrize = ChordLength );

    /// Return a degree 3 or less spline passing through the points of 'passThrough' (size>1). 'internalT' (size=sizeof(passThrough)-2) specifies the
    /// increasing T values of the internal points of 'passThrough' (the first of which has T value 0 and the last of which has T value 1).
    static std::unique_ptr< BSpline2 > naturalInterpolation(
        const std::vector< Vector2 >& passThrough, const std::vector< double >& internalT );
    /// Use chord-length parametrization to come up with 'internalT' for the overload.
    static std::unique_ptr< BSpline2 > naturalInterpolation( const std::vector< Vector2 >& passThrough );
    static std::vector< double > naturalInterpolationChordLengthT( const std::vector< Vector2 >& passThrough );

    /// Create a uniform spline from 'degree' and 'control', then add multiple knots by duplicating the knots
    /// referenced by 'knotsToDuplicate', whose indices refer to intermediate ("internal") knots (duplication of end knots is forbidden). This is intended
    /// for generating unit-test data.
    static UniquePtr splineWithMultipleKnots(
        int degree, const Control& control, std::vector< int > intermediateKnotsToDuplicate );

    /// Determine how many subcurves 'spline' could be split into--subcurves that are possibly no more than C0
    /// with each other--by locating knots of multiplicity 'spline.degree' in 'spline'. Return the split times (the multiple knots).
    static std::vector< double > c0Times( const Type& spline );

    /// Return the number of internal knots associated with a spline with the indicated properties. 'numControl' must be at least 'degree + 1'.
    static size_t numInternalKnots( int degree, size_t numControl );

    /// Create a degree-1 spline with the provided control points.
    static UniquePtr polyline( const Control& );
    /// Create a closed box curve.
    static UniquePtr boxPolyline( const BoundingBoxd& );
    static UniquePtr lineSeg( const Vector2& a, const Vector2& b );
    static UniquePtr lineSeg( const LineSegment& );
    static UniquePtr spline( int degree, const Control& control );
    /// See createFromControlPointsAndKnots.
    static UniquePtr spline( int degree, const Control& control, const std::vector< double >& intermediateKnots );

    /// The default number of samples to use when approximating the length of a spline.
    static const size_t defaultLengthPrecision = 20;
private:
    /// <X> From Geometric Tools Library.
    using GTESpline = gte::BSplineCurve< 2, double >;

    BSpline2();

    // ltwarning: This crashes on GTE failure (cassert).
    static Control controlFitToDataPoints(
        int degree,
        int numControlPoints,
        const Control& dataPoints,
        CurveFitParametrizeType parametrize );

    Control _controlPoints;
    int _degree;
    std::unique_ptr< GTESpline > _gteSpline;
    mutable size_t _cachedPrecision;
    mutable double _cachedLength;
};

} // core

#endif // #include guard
