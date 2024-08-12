#ifndef MASHUP_DRAWINGID_H
#define MASHUP_DRAWINGID_H

namespace mashup {

/// Ordinally identify one of the 'Drawing's in a blend scenario.
enum DrawingID
{
    DrawingA = 0,
    DrawingB = 1,
    NumDrawings = 2
};

/// 'id' < 'NumDrawings'
DrawingID otherDrawing( DrawingID id );

} // mashup

#endif // #include
