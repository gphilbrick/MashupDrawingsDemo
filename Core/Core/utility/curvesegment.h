#ifndef CORE_CURVESEGMENT_H
#define CORE_CURVESEGMENT_H

#include <Core/utility/boundingbox.h>
#include <Core/utility/curvecurveintersection.h>
#include <Core/utility/vector2.h>

#include <map>
#include <set>

namespace core {

class BSpline2;

/// A utility object to facilitate curve-curve intersection code between curves A and B.
struct CurveSegment
{
    /// Two unequal indices in a CurveSegments.
    using PairToCheck = std::pair< size_t, size_t >;
    using PairsToCheck = std::set< PairToCheck >;

    // Kill the default copy operator.
    CurveSegment( bool aOrBVal );
    CurveSegment();

    /// Set 'store' to represent the entirety of 'curve', which is identified as either A or B via 'aOrB'.
    static void wholeCurveToSegment( const BSpline2& curve, bool aOrB, CurveSegment& store );

    bool aOrB; // true means a.
    BoundingBoxd bounds;
    std::vector< Vector2 > control;
    BoundingIntervald tInterval;
};
using CurveSegments = std::vector< CurveSegment >;

} // core

#endif // #include
