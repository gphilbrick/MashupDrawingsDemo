#include <drawings.h>

namespace mashup {

DrawingID Drawings::whichDrawing( StrokeHandle s ) const
{
    if( ( *this )[ DrawingID::DrawingA ].contains( s ) ) {
        return DrawingID::DrawingA;
    } else if( ( *this )[ DrawingID::DrawingB ].contains( s ) ) {
        return DrawingID::DrawingB;
    } else {
        return DrawingID::NumDrawings;
    }
}

} // mashup
