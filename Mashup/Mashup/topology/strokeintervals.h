#ifndef MASHUP_TOPOLOGY_STROKEINTERVALS_H
#define MASHUP_TOPOLOGY_STROKEINTERVALS_H

#include <Mashup/tinterval.h>

namespace mashup {

struct Substroke;

namespace topology {

/// For some 'Stroke' 's', break the [0,1] T space into occluded and unoccluded intervals (for
/// the interests of topology).
class StrokeIntervals
{
public:
    /// 'unoccluded' contains 0 or more T intervals in [0,1], ordered by increasing T, specifying
    /// where 's' is unoccluded. The intervals must not overlap and must each have nonzero size.
    StrokeIntervals( const TIntervals& unoccluded );
    /// Create a single unoccluded interval covering [0,1]
    StrokeIntervals();
    TIntervals intervals( bool occluded ) const;
    bool anyUnoccluded() const;
    /// Given 'occludedIdx', which must index one of the occluded intervals, store
    /// in 'storeBefore' and 'storeAfter' the indices of the unoccluded intervals before
    /// and after 'occludedIdx' (if any).
    void unoccludedAdjacentTo( size_t occludedIdx,
                               boost::optional< size_t >& storeBefore,
                               boost::optional< size_t >& storeAfter ) const;
    /// Assuming 'ss' is or lies inside one of the unoccluded intervals of 's', return
    /// the 's'-relative index of the 'Crossing' that 'ss' points toward, or boost::none
    /// if 'ss' does not point toward a 'Crossing'.
    boost::optional< size_t > crossingIndex( const Substroke& ss ) const;
private:
    /// In (0,1)
    std::vector< double > _splitT;
    bool _occludedAtZero = false; // as in at T=0
    size_t _numOccluded = 0;
    size_t _numUnoccluded = 0;
};

} // topology
} // mashup

#endif // #include
