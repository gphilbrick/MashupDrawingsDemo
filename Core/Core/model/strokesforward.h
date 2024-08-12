#ifndef CORE_MODEL_STROKESFORWARD_H
#define CORE_MODEL_STROKESFORWARD_H

#include <memory>
#include <vector>

namespace core {
namespace model {

class Stroke;

using UniqueStroke = std::unique_ptr< Stroke >;
using UniqueStrokes = std::vector< UniqueStroke >;
using RawConstStrokes = std::vector< const Stroke* >;

} // model
} // core

#endif // #include
