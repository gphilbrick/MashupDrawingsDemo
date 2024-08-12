#ifndef MASHUP_CHAINS_JOINER_H
#define MASHUP_CHAINS_JOINER_H

#include <Mashup/strokeforward.h>

#include <memory>

namespace mashup {

namespace topology {
class Crossing;
} // topology

class BlendDrawings;
struct Substroke;

namespace chains {

struct NextStep;

/// An interface for determining which 'Stub's of some 'Crossing' 'c' can be connected and for
/// choosing which 'Stub' pairs to connect.
class Joiner
{
public:
    using Barrier = Stroke;
    using Stub = Substroke;

    /// Create a 'Joiner' meant for figuring out how to connect the stubs of 'c'.
    Joiner( const topology::Crossing& c, const BlendDrawings& );
    ~Joiner();

    /// Given what has/hasn't been joined yet at 'c', return whether it's possible to
    /// make a connection between 'a' and 'b', which must both be original, untrimmed 'Stub's (directed into 'c').
    bool canJoin( const Stub& a, const Stub& b ) const;
    /// 'a' and 'b', which are original untrimmed 'Stub's of 'c', must currently be join-able as per 'canJoin'. Return non-null instructions
    /// as to how to move from 'a' to 'b' along a chain.
    std::unique_ptr< NextStep > join( const Stub& a, const Stub& b );
    /// 'stub' is an original, untrimmed 'Stub' from 'c'.
    /// Assuming 'stub' isn't joined to anything, return a version of 'stub' that has been "pretrimmed" to make way
    /// for those connections that _have_ been finalized via join().
    Stub pretrimmed( const Stub& stub ) const;
private:
    struct Imp;
    const std::unique_ptr< Imp > _imp;
};

} // chains
} // mashup

#endif // #include
