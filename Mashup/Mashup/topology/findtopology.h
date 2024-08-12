#ifndef MASHUP_TOPOLOGY_FINDTOPOLOGY_H
#define MASHUP_TOPOLOGY_FINDTOPOLOGY_H

#include <Mashup/drawing.h>

#include <map>
#include <memory>

namespace mashup {

struct StrokePoly;

namespace topology {

class Topology;

class FindTopology
{
public:
    using StrokeToPoly = std::map< StrokeHandle, StrokePoly >;
    FindTopology( const Drawing& a, const Drawing& b, const StrokeToPoly& );
    ~FindTopology();
    std::unique_ptr< Topology > topology();
private:
    struct Imp;
    const std::unique_ptr< Imp > _imp;
};

} // topology
} // mashup

#endif // #include
