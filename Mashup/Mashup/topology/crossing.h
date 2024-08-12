#ifndef MASHUP_TOPOLOGY_CROSSING_H
#define MASHUP_TOPOLOGY_CROSSING_H

#include <Mashup/substroke.h>

#include <boost/optional.hpp>

#include <map>

namespace mashup {
namespace topology {

/// A location in a blend-drawings scenario where 2 or more 'Stroke's (not all
/// from the same drawing) cross each other.
class Crossing
{
public:
    /// A 'Substroke' that ends at 'this' (at the crossing represented by 'this').
    using Stub = Substroke;

    Crossing();

    /// 's' must end at the crossing.
    void add( const Stub& s );
    /// 'a' and 'b' must be "co-stubs" representing the two pieces of a 'Stroke'
    /// passing through the crossing. They must both end at the crossing, e.g.,
    /// one might pass in [0,0.3] and [0.5,0.3] of some 'Stroke' s where s enters
    /// 'this' at T=0.3.
    void add( const Stub& a, const Stub& b );
    /// 'env' represents (a) a 'Stub' entering 'this', the occluded part of that 'Stroke'
    /// inside of 'this', and possibly (c) the unoccluded piece of 'Stroke' heading out of 'this'
    void addEnvelopeAroundOccluded( const Substroke& env );

    /// Return all the 'Substroke's feeding into this, all oriented so that
    /// they end where they enter the crossing.
    const std::vector< Stub >& stubs() const;
    /// Return whether 'a' and 'b' (which must have been 'add'ed already), represent a single
    /// interval on some original-drawing 'Stroke'. 'a' and 'b' should be "stubs" of 'this' (must be oriented toward each other/toward
    /// the crossing they share).
    bool originallyConnected( const Stub& a, const Stub& b ) const;
    boost::optional< Stub > originalConnection( const Stub& ) const;
    /// Return whether the point 't' on 's' is part of one of the 'Stub's heading into 'this' or
    /// part of one of the 'Substroke's occluded at 'this'.
    bool isPartOf( StrokeHandle s, double t ) const;
private:
    /// All of these end at the crossing. These are the "roads" heading into the 'Crossing'.
    std::vector< Stub > _stubs;
    /// These collectively represent everything in '_stubs', _plus_ occluded intervals.
    std::vector< Substroke > _envelopesAroundOccluded;
    std::map< Stub, Stub, Stub::CompFunc > _originalConnections;
};

} // topology
} // mashup

#endif // #include
