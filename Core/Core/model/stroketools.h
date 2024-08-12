#ifndef CORE_MODEL_STROKETOOLS_H
#define CORE_MODEL_STROKETOOLS_H

#include <Core/model/posforward.h>
#include <Core/model/stroke.h>

#include <boost/optional.hpp>

#include <memory>
#include <vector>

namespace core {
namespace model {

/// Return whether 's' is simple enough that we can approximate its outline with a rotated box.
bool isSimpleSegStroke( const Stroke& s );
UniqueStroke simpleSegStroke( const model::Pos& start, const model::Pos& end, double width );

UniqueCurve constWidthCurve( double width );
UniqueStroke multiplyStrokeWidth( const Stroke&, double factor );
UniqueStroke multiplyStrokeWidth( const Stroke&, const Curve& widthCurve );

UniqueStroke strokeFromPosAndWidth( UniqueCurve&& posToOwn, UniqueCurve&& widthToOwn );

/// 'width' should be twice the radius of an intended circle.
UniqueStroke lineSegStroke( const model::Pos& posA, const model::Pos& posB, double width );
UniqueStroke circleStroke( const model::Pos& center, double rad, double strokeWidth );

/// Return the product of the two curves (think of one as attenuating the other). The result
/// will be defined over the same X interval as 'toChange'.
UniqueCurve multiplyWidthCurves( const Curve& toChange, const Curve& multiplyBy );
void multiplyWidthCurve( Curve& width, double factor );

/// The returned 'Stroke' will have its supporting data taken from the first item in 'strokes'.
/// If 'strokes' is empty, return 'nullptr'.
UniqueStroke stitchC0Strokes(
    const std::vector< const Stroke* >& parts,
    bool loop = false,
    std::vector< double >* storePartEndT = nullptr );
UniqueStroke stitchC0Strokes(
    const std::vector< UniqueStroke >& parts,
    bool loop = false,
    std::vector< double >* storePartEndT = nullptr );
UniqueStroke stitchC0Strokes( const Stroke& a, const Stroke& b, double* storeStitchT = nullptr );
UniqueStroke stitchC0Strokes(
    const std::vector< const Stroke* >& strokes,
    bool loop,
    const std::vector< double >& partWeights,
    std::vector< double >* storePartEndT = nullptr );
std::vector< double > partWeightsForC0Stitch( const std::vector< const Stroke* >& strokes );

/// Return whether the composite 'Stroke' represented by 'parts' and 'loop' involves an approximately C0
/// stitch (no endpoints that ought in principle to be equal are further than 'maxErrorDist' from each
/// other).
bool strokesAreApproxC0( const std::vector< const Stroke* >& parts, bool loop, double maxErrorDist );
bool strokesAreApproxC0( const std::vector< UniqueStroke >& parts, bool loop, double maxErrorDist );

/// Wrap core::BSpline2Utility::stitchC0Spline, first changing the X coordinates of 'parts' so that they make
/// more sense for debugging purposes (they are not functionally important).
UniqueCurve stitchC0WidthCurve(
    const std::vector< const Curve* >& parts,
    const std::vector< double >& tWeights );

/// Return a copy of 'original' with its X values shifted to fill the interval [xStart,xEnd], or
/// return 'original' itself if it is vertical.
UniqueCurve setXIntervalForWidthCurve(
    const Curve& original, double xStart, double xEnd );

UniqueCurve linearWidthCurve( double widthStart, double widthEnd );

/// Return the result of assigning 'widthCurve' as the width curve of the stroke.
UniqueStroke setWidthCurve( const Stroke&, UniqueCurve&& widthCurve );

/// Return a version of 's' with one or both of its endpoints tapered to width=0. If 'tA' is set, the start of
/// 's' should be tapered to zero, with the taper ending at 'tA'. If 'tB' is set, the end of 's' should be
/// tapered to zero, with the taper starting at 'tB'. If 'tA' > 'tB', overlap the tapers.
///
/// 'tA' or 'tB' must be set.
UniqueStroke taperStrokeEndpoints( const Stroke& s, boost::optional< double > tA, boost::optional< double > tB );

} // model
} // core

#endif // #include
