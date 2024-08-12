#ifndef CORE_CURVECURVEINTERSECTION_H
#define CORE_CURVECURVEINTERSECTION_H

#include <Core/utility/boundingbox.h>

#include <vector>

namespace core {

/// An approximate intersection between curves A and B.
struct CurveCurveIntersection
{
    CurveCurveIntersection converse() const
    {
        CurveCurveIntersection reversed;
        reversed.tIntervalA = tIntervalB;
        reversed.tIntervalB = tIntervalA;
        reversed.hitBox = hitBox;
        return reversed;
    }

    BoundingIntervald tIntervalA;
    BoundingIntervald tIntervalB;

    // The intersection falls somewhere in this box.
    BoundingBoxd hitBox;
};
using CurveCurveIntersections = std::vector< CurveCurveIntersection >;

} // core

#endif // #include
