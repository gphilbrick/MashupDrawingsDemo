#ifndef PRINTCURVES_CAP_H
#define PRINTCURVES_CAP_H

#include <Core/utility/vector2.h>

#include <boost/optional.hpp>

namespace printCurves {

/// Details about how to render the end of a "varying-width curve."
struct Cap
{
    Cap() {}

    Cap( const core::Vector2& wallNormalVal ) : wallNormal( wallNormalVal )
    {
    }

    Cap( const boost::optional< core::Vector2 >& wallNorm ) : wallNormal( wallNorm )
    {
    }

    /// If this is set, this end of the stroke should be rendered to be flush with
    /// a "wall" having this normal. The normal should point into the face of a traveler
    /// moving along the curve toward the terminus identified with this Cap.
    boost::optional< core::Vector2 > wallNormal;
};

} // printCurves

#endif // #include
