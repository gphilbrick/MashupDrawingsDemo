#ifndef CORE_MODEL_POSFORWARD_H
#define CORE_MODEL_POSFORWARD_H

// This is meant to consolidate references to core::Vector2 in this project. It's a late addition, though;
// there are currently tons of explicit "core::Vector2"s in the vink namespace.

namespace core {
class Vector2;
} // core

namespace core {
namespace model {

using Pos = core::Vector2;

} // model
} // core

#endif // #include
