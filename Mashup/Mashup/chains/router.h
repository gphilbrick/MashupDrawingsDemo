#ifndef MASHUP_ROUTER_H
#define MASHUP_ROUTER_H

#include <Mashup/chains/joint.h>
#include <Mashup/substroke.h>

#include <boost/optional.hpp>

#include <memory>

namespace mashup {

class BlendDrawings;
class RandomBinary;

namespace topology {
class Crossing;
} // topology

namespace chains {

struct NextStep;

/// Decides which 'Substroke's connect to which at a 'Crossing', and how (what kind of trimming, joint is involved).
class Router
{
public:
    using Crossing = topology::Crossing;

    Router( const Crossing&, const BlendDrawings&, RandomBinary& );
    ~Router();

    /// Let 'prev' be an original, untrimmed 'Substroke' from 'Topology'.
    /// Assuming 'prev' is part of a chain, return how to construct the part of the chain including (some of) 'prev' along
    /// with some next 'Substroke'. If 'prev' represents the end of a chain, return nullptr.
    ///
    /// If the chain ends with 'prev' (return nullptr) and 'prev' needs to be cut off prematurely based on
    /// "pretrimming" within 'Joiner', then set 'prevPretrimmed' to the pretrimmed version of 'prev'
    std::unique_ptr< NextStep > next( const Substroke& prev, boost::optional< Substroke >& prevPretrimmed ) const;
private:
    struct Imp;
    const std::unique_ptr< Imp > _imp;
};

} // chains
} // mashup

#endif // #include
