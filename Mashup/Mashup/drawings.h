#ifndef MASHUP_DRAWINGS_H
#define MASHUP_DRAWINGS_H

#include <Mashup/drawing.h>
#include <Mashup/drawingid.h>

#include <array>

namespace mashup {

/// All the 'Drawing's in a blend scenario.
class Drawings : public std::array< Drawing, DrawingID::NumDrawings >
{
public:
    /// Return which of the 'Drawing's 's' comes from, or
    /// DrawingId::NumDrawings if neither.
    DrawingID whichDrawing( StrokeHandle s ) const;
};

} // mashup

#endif // #include
