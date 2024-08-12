#ifndef MASHUP_SCALARFIELD_H
#define MASHUP_SCALARFIELD_H

#include <Core/math/canvasfield.h>

namespace mashup {

class ScalarField : public core::math::CanvasField< double >
{
public:
    using Parent = core::math::CanvasField< double >;
    ScalarField( const BoundingBox& bounds, double initialVal );
};

} // mashup

#endif // #include
