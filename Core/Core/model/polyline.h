#ifndef CORE_MODEL_POLYLINE_H
#define CORE_MODEL_POLYLINE_H

#include <Core/model/posback.h>

#include <Core/utility/vector2.h>

#include <vector>

namespace core {
namespace model {

using Polyline = std::vector< Pos >;
using Polylines = std::vector< Polyline >;

} // model
} // core

#endif // #include
