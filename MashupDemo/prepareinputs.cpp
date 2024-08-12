#include <prepareinputs.h>

#include <mashupinputs.h>

#include <Core/model/stroketools.h>

namespace mashupDemo {

const double canvasDim = 1000.;

MashupInputs prepareScenario_basic()
{
    MashupInputs ret;
    ret.name = "basic";
    ret.canvasBounds = core::BoundingBoxd{
        core::Vector2( 0, 0 ),
        core::Vector2( canvasDim, canvasDim ) };

    const auto& box = ret.canvasBounds;

    const auto c_uv = 0.4;
    const auto radMax = box.maxDim() * 0.35;
    const auto radMin = box.maxDim() * 0.2;
    const auto widthMax = box.maxDim() * 0.02;
    const auto widthMin = box.maxDim() * 0.008;

    const auto placeCircles = [ & ](
        mashup::Drawing& drawing,
        const core::model::Pos& center,
        size_t numCircles )
    {
        for( size_t i = 0; i < numCircles; i++ ) {
            const auto f = F_FROM_I( i, numCircles );
            drawing.addStroke(
                core::model::circleStroke(
                    center,
                    core::mathUtility::lerp( radMax, radMin, f ),
                    core::mathUtility::lerp( widthMin, widthMax, f ) ) );
        }
    };

    placeCircles(
        ret.inputDrawings[ mashup::DrawingID::DrawingA ],
        box.posFromUV( c_uv, c_uv ),
        3 );
    placeCircles(
        ret.inputDrawings[ mashup::DrawingID::DrawingB ],
        box.posFromUV( 1. - c_uv, 1. - c_uv ),
        3 );

    return ret;
}

MashupInputs prepareScenario_preserveA()
{
    auto ret = prepareScenario_basic();
    ret.name = "preserveA";
    ret.options.preserveDrawing = mashup::DrawingID::DrawingA;
    return ret;
}

MashupInputs prepareScenario_useABField()
{
    MashupInputs ret;
    ret.name = "useABField";
    ret.canvasBounds = core::BoundingBoxd{
        core::Vector2( 0, 0 ),
        core::Vector2( canvasDim, canvasDim ) };

    const double margin = 0.1;
    const core::BoundingBoxd gridBounds
    {
        ret.canvasBounds.posFromUV( margin, margin ),
        ret.canvasBounds.posFromUV( 1. - margin, 1. - margin ),
    };
    const size_t strokesWide = 10;

    const auto placeGridOfStrokes = [ & ](
        mashup::Drawing& drawing,
        const core::model::Pos& offset,
        double strokeRad,
        double strokeWidth ) {
        for( size_t x = 0; x < strokesWide; x++ ) {
            const auto fX = F_FROM_I( x, strokesWide );
            for( size_t y = 0; y < strokesWide; y++ ) {
                const auto fY = F_FROM_I( y, strokesWide );
                const auto strokeCenter = offset + gridBounds.posFromUV( fX, fY );
                drawing.addStroke( core::model::circleStroke( strokeCenter, strokeRad, strokeWidth ) );
            }
        }
    };

    const auto rad = ( gridBounds.avgDim() / static_cast< double >( strokesWide ) ) * 0.4;
    placeGridOfStrokes(
        ret.inputDrawings[ mashup::DrawingID::DrawingA ],
        core::model::Pos{ 0., 0. },
        rad,
        rad * 0.1 );
    const auto off = rad * 0.5;
    placeGridOfStrokes(
        ret.inputDrawings[ mashup::DrawingID::DrawingB ],
        core::model::Pos{ off, off },
        rad * 0.7,
        rad * 0.2 );

    // Set up this scenario to use an AB field that completely favors Drawing A on the left side of 'ret.canvasBounds'
    // and completely favors Draqwing B on the right side.
    {
        using ABF = mashup::ABField;
        ret.abField = std::make_unique< ABF >( ret.canvasBounds, 0. );

        // Linear horizontal gradient from field=1. to field=0., left to right.
        ABF::Gradient grad;
        grad.alpha = 1.;
        grad.posA = ret.canvasBounds.posFromUV( 0.2, 0. );
        grad.posB = ret.canvasBounds.posFromUV( 0.8, 0. );
        grad.type = ABF::GradientType::Linear;
        grad.val = 1.;
        ret.abField->applyGradient( grad );

        ret.options.routing.wFunctor =
            std::make_unique< mashup::WeightFunctor_ABFieldBased >( *ret.abField );
    }

    // Tailor options to this more finely detailed scenario.
    ret.options.tails.maxRad_canvas = 20.;
    ret.options.routing.jointRad = 20.;

    return ret;
}

} // mashupDemo
