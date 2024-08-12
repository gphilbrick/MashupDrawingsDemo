#ifndef MASHUP_ABFIELD_H
#define MASHUP_ABFIELD_H

#include <Mashup/scalarfield.h>

namespace mashup {

/// A scalar [0,1] field over the visible canvas (not necessarily
/// the bounding box of 'BlendDrawings').
///
/// A value of 1. means that drawing-A 'Stroke's have weight 1. and
/// drawing-B 'Stroke's have weight 0., whereas a value of 0. means
/// the opposite.
using ABField = ScalarField;

} // mashup

#endif // #include
