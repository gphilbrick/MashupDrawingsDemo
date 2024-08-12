#ifndef MASHUP_CHAINS_CHAIN_H
#define MASHUP_CHAINS_CHAIN_H

#include <Mashup/chains/joint.h>
#include <Mashup/substroke.h>

#include <boost/noncopyable.hpp>

#include <array>
#include <memory>
#include <vector>

namespace mashup {
namespace chains {

/// A chain of substrokes with joints in between _some_ of them.
struct Chain : private boost::noncopyable
{    
    bool hasTail( bool startOrEnd ) const;
    /// Collapse any sequence of 2 or more 'substroke's that represents
    /// a single interval on some 'Stroke'.
    std::unique_ptr< Chain > simplified() const;
    std::unique_ptr< Chain > clone() const;

    std::unique_ptr< Stroke > stroke() const;

    // for debugging
    bool hasBadJoint( double maxEndpointMismatch, double* storeBadDist = nullptr ) const;

    /// 'a' and 'b' together should represent one 'Chain' (the first substroke of 'a' should be reverse of first of 'b').
    /// If one is closed, the other should be a dummy 'Chain' (one substroke, open).
    /// Check that these conditions apply (throws std::runtime_error otherwise) and return the single 'Chain'
    /// that 'a' and 'b' together represent.
    static std::unique_ptr< Chain > checkAndCombineHalves( const Chain& a, const Chain& b );    

    /// Oriented consistently (end of ith lies at start of i+1th, or at start of ((i+1)%numSubstrokes) if 'closed' is true).
    std::vector< Substroke > substrokes;

    /// length( substrokes ) - 1  if 'closed' is false
    /// length( substrokes )      if 'closed' is true
    ///
    /// If 'substroke's i and i + 1 represent a single interval on the same 'Stroke',
    /// then 'joints[ i ]' is nullptr.
    std::vector< Joint > joints;

    bool closed = false;
};
using UniqueChain = std::unique_ptr< Chain >;
using UniqueChains = std::vector< UniqueChain >;

} // chains
} // mashup

#endif // #include
