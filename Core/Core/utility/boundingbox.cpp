#include "boundingbox.h"

namespace core {

BoundingBoxi boundingBoxFloatToInt(const BoundingBoxd& b)
{
    return BoundingBoxi((int)b.xMin(),(int)b.yMin(),(int)b.xMax(),(int)b.yMax());
}

BoundingBoxd boundingBoxIntToFloat(const BoundingBoxi& b)
{
    return BoundingBoxd(b.xMin(),b.yMin(),b.xMax(),b.yMax());
}

} // core
