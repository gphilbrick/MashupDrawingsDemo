#include <scalarfield.h>

namespace mashup {

ScalarField::ScalarField( const BoundingBox& bounds, double initialVal )
    : Parent( bounds, 100, initialVal )
{
}

} // mashup
