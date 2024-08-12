#ifndef CORE_MODEL_BOUNDINGBOXFORWARD_H
#define CORE_MODEL_BOUNDINGBOXFORWARD_H

namespace core {
class Vector2;
template< typename T, typename TCoords >
class BoundingBox;
using BoundingBoxd = BoundingBox< double, Vector2 >;
}

namespace core {
namespace model {
using BoundingBox = core::BoundingBoxd;
} // model
} // core

#endif // #include
