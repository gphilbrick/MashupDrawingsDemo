#ifndef MASHUP_CHAINS_NEXTSTEP_H
#define MASHUP_CHAINS_NEXTSTEP_H

#include <Mashup/chains/joint.h>
#include <Mashup/substroke.h>

namespace mashup {
namespace chains {

/// Let (implicit) 'prev' be some 'Substroke' taken (untrimmed) from
/// 'Topology'. Imagine 'prev' to be a part of a chain (again, before
/// trimming to accommodate joints).
///
/// This indicates how to take the next step in the chain.
struct NextStep
{
    std::unique_ptr< NextStep > clone() const;
    std::unique_ptr< NextStep > reverse( const Substroke& prev ) const;

    /// Return a 'Stroke' representing the "middle" part of 'this', which is either
    /// (a) a copy of 'joint' (if 'joint' non-null)
    /// (b) the part of the 'Stroke' on which 'prevTrimmed' and 'nextTrimmed' lie, taken between these two.
    std::unique_ptr< Stroke > midStroke() const;
    /// Return the whole sequence from 'prev' to 'next' as a 'Stroke'
    std::unique_ptr< Stroke > asStroke() const;

    /// Assuming 'stubA' and 'stubB' are two stubs from the same 'Crossing' (both point into it), which
    /// together can be interpreted as a single interval on the same 'Stroke', return a joint-less
    /// 'NextStep' representing going from 'stubA' to 'stubB' along the chain containing 'stubA'.
    static std::unique_ptr< NextStep > singleIntervalCase( const Substroke& stubA, const Substroke& stubB );

    /// Same as 'prev' if 'joint' is null; otherwise ends earlier to accommodate 'joint'.
    Substroke prevTrimmed;

    /// The next step in the chain, before trimming to accommodate non-null 'joint'.
    /// Oriented along with 'prev', i.e., starts at the 'Crossing' shared with 'prev' and heads out of said 'Crossing'.
    Substroke next;
    /// Same as 'next' if 'joint' is null; otherwise starts later to accommodate 'joint'.
    Substroke nextTrimmed;

    /// If null, then 'prev' and 'next' can be considered to represent a single interval
    /// on the same 'Stroke', and 'prevTrimmed' and 'nextTrimmed' may be ignored.
    /// If not-null, then the actual chain we're building will contain the sequence
    /// 'prevTrimmed', 'joint', 'nextTrimmed'. ('prev' and 'next'
    Joint joint;
};

} // chains
} // mashup

#endif // #include
