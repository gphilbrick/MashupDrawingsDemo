#ifndef MASHUP_TOPOLOGY_TOPOLOGY_H
#define MASHUP_TOPOLOGY_TOPOLOGY_H

#include <Mashup/chains/chain.h>
#include <Mashup/strokeforward.h>
#include <Mashup/topology/strokeintervals.h>

#include <boost/noncopyable.hpp>

#include <memory>

namespace mashup {

struct BlendOptions;
class Drawings;

namespace topology {

class Crossing;
struct StrokeIntersection;

/// The layout of a drawing-blend in terms of 'Substroke's and the 'Crossing's they meet at.
class Topology
{
public:    
    // The reason for this clarification is that chain-building involves trimming 'Substroke's to
    // accommodate joints. These trimmed pieces currently cannot be successfully used in lookup
    // calls to 'this'.
    /// One of the original, untrimmed 'Substroke's recognized internally as keys by
    /// 'this', OR a reverse() copy of one of the foregoing.
    ///
    /// In other words, this is one of the 'Substroke's returned by unoccludedSubstrokes() or
    /// a reverse() copy of one of these.
    ///
    /// An interval (other than [0,1] or [1,0]) on an 'UntrimmedOriginalSS' is NOT an 'UntrimmedOriginalSS'.
    using OriginalSubstroke = Substroke;

    Topology();
    ~Topology();

    /// FOR BUILDING

    /// Add a 'Stroke', characterized by which intervals are occluded and which are unoccluded.
    /// Do not add the same 'Stroke' more than once.
    /// 'intervals' must be valid (increasing order, fully covers [0,1], none of the intervals overlap).
    void addStroke( const Stroke&, const StrokeIntervals& intervals );
    /// Call after all 'addStroke' calls. If either of the indicated 'Stroke's is not recognized by 'this',
    /// do nothing.
    void addStrokeIntersection( const StrokeIntersection& );
    /// Signal that no more information is to be added.
    void doneAdding();

    /// FOR USING (after 'doneAdding()')

    // The purpose of 'd' is to have consistency from run to run instead of getting different orderings
    // based on 'Stroke' pointers.
    /// Return all the unoccluded 'Substroke's in the scenario, using 'd' for ordering.
    std::vector< OriginalSubstroke > unoccludedSubstrokes( const Drawings& d ) const;

    /// Return whether 'a' and 'b' represent what used to be a single interval on some original-drawing 'Stroke'.
    /// 'a' and 'b' must be consistently oriented (e.g., they might be [0.4, 0.5] and [0.51, 0.6] on some 'Stroke').
    ///
    /// Does not deal with 'a' and 'b' that are not connected at a 'Crossing' but do represent T=0 and T=1 of the
    /// a closed 'Stroke'.
    ///
    /// Call after chains().
    bool originallyConnected( const OriginalSubstroke& a, const OriginalSubstroke& b ) const;

    /// Return handles to all the 'Crossing's internally maintained.
    std::vector< const Crossing* > crossings() const;

    /// Return a handle to the 'Crossing' which 'ss' terminates at (if it terminates at a 'Crossing'; return
    /// nullptr if it doesn't). 'ss' must be or lie inside one of the "unoccluded 'Substroke's" of 'this'
    /// (one of the 'Substroke's returned by unoccludedSubstrokes() ).
    const Crossing* findCrossing( const Substroke& ss ) const;
private:
    struct Imp;
    const std::unique_ptr< Imp > _imp;
};

} // topology
} // mashup

#endif // #include
