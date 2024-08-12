#ifndef CORE_INTERSECTIONPARAMETERS_H
#define CORE_INTERSECTIONPARAMETERS_H

namespace core {

/// Indicates what degree(s) of approximation to use when finding the intersections between two curves.
struct IntersectionParameters
{
    IntersectionParameters( double minBoxDimVal, double minDistBetweenIntersectionsVal ) :
        minBoxDim( minBoxDimVal ),
        minDistBetweenIntersections( minDistBetweenIntersectionsVal )
    {}
    /// If a bounding box has width and height below this value, that bounding box is considered a candidate intersection.
    double minBoxDim;
    /// If a new intersection is found, it must be at least this distance away (in Cartesian sense, not T sense) from
    /// other already found intersections.
    double minDistBetweenIntersections;
};

} // core

#endif // #include
