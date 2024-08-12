#ifndef MASHUP_SAMEDRAWINGHITS_H
#define MASHUP_SAMEDRAWINGHITS_H

#include <Mashup/drawingid.h>
#include <Mashup/strokeforward.h>

#include <boost/optional.hpp>

#include <array>
#include <map>
#include <set>

namespace mashup {

namespace topology {
class Crossing;
} // topology

struct Substroke;

/// Records where original 'Stroke's within some 'Drawing' 'd' hit each other (including
/// 'Stroke' self-hits
class SameDrawingHits
{
public:
    void addHit( StrokeHandle a, StrokeHandle b, double tA, double tB );

    /// Return the T value in 'ss' of the first or last place (as per 'firstOrLast') that 'ss' undergoes some
    /// same-'Drawing' intersection with another original 'Stroke 'b' (where 'b' at T-of-'b'-at-intersection
    /// is not part of 'ignoreIn). If there is no same-'Drawing' intersection on 'ss' that isn't excluded by
    /// 'ignoreIn', return boost::none.
    boost::optional< double > firstOrLastHit( const Substroke& ss, bool firstOrLast, const topology::Crossing& ignoreIn ) const;
    void clear();
private:
    struct Hit
    {
        double t = 0.;
        double tOther = 0.;
        StrokeHandle other = nullptr;

        bool operator < ( const Hit& ) const;
    };
    using Hits = std::set< Hit >;
    std::map< StrokeHandle, Hits > _strokeToHits;
};

using DrawingToSameDrawingHits = std::array< SameDrawingHits, DrawingID::NumDrawings >;

} // mashup

#endif // #include
