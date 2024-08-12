#include <drawingid.h>

namespace mashup {

DrawingID otherDrawing( DrawingID id )
{
    return id == DrawingA ? DrawingB : DrawingA;
}

} // mashup
