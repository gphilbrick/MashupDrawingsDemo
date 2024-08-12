#include <weightfunctor.h>

#include <abfield.h>
#include <blenddrawings.h>
#include <drawings.h>
#include <strokeback.h>

#include <Core/model/curveback.h>
#include <Core/exceptions/runtimeerror.h>

namespace mashup {

double WeightFunctor_WidthBased::weight( const Stroke& s, StrokeT t, const BlendDrawings& ) const
{
    return s.width( t );
}

WeightFunctor_ABFieldBased::WeightFunctor_ABFieldBased(
    const ABField& abField )
    : _abField( abField )
{
}

double WeightFunctor_ABFieldBased::weight( const Stroke& s, StrokeT t, const BlendDrawings& bd ) const
{
    const auto& drawings = bd.drawings();
    const auto canvasPos = s.curve().position( t );
    const auto sAffil = drawings.whichDrawing( &s );
    if( sAffil == DrawingID::NumDrawings ) {
        THROW_UNEXPECTED;
    }
    const auto aDomAtPos = _abField.interp( canvasPos );
    return sAffil == DrawingID::DrawingA
        ? std::clamp( aDomAtPos, 0., 1. )
        : std::clamp( 1. - aDomAtPos, 0., 1. );
}

} // mashup
