#ifndef MASHUP_CHAINS_CHAINBUILDER_H
#define MASHUP_CHAINS_CHAINBUILDER_H

#include <Mashup/chains/chain.h>

#include <memory>

namespace core {
namespace view {
class ProgressBar;
} // view
} // core

namespace mashup {

class BlendDrawings;

namespace topology {
class Topology;
} // topology

namespace chains {

class ChainBuilder
{
public:
    ChainBuilder( const topology::Topology&, const BlendDrawings&, core::view::ProgressBar* );
    ~ChainBuilder();
    /// If one of the 'Drawing's is to be preserved as per 'BlendOptions',
    /// none of the returned 'Chain's may contain 'Substroke's from said
    /// 'Drawing'.
    UniqueChains chains();
private:
    struct Imp;
    const std::unique_ptr< Imp > _imp;
};

} // chains
} // mashup

#endif // #include
