#ifndef MASHUP_TOPOLOGY_STROKEINTERSECTION_H
#define MASHUP_TOPOLOGY_STROKEINTERSECTION_H

#include <Mashup/strokeforward.h>

#include <array>

namespace mashup {
namespace topology {

struct StrokeIntersection
{
    std::array< const Stroke*, 2 > stroke;
    std::array< double, 2 > t;
};

} // topology
} // mashup

#endif // #include
