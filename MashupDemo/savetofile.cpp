#include <savetofile.h>

#include <Core/model/rgbback.h>
#include <Core/model/stroke.h>
#include <Core/model/stroke.h>

#include <PrintCurves/curvespostscript.h>
#include <PrintCurves/createfiles.h>

namespace mashupDemo {

namespace {

using Curve = core::model::Curve;
using CPS = printCurves::CurvesPostScript;
using RGB = core::model::RGB;

const double defaultMinPsDim = 1000.0; // Minimum PostScript dimension
const double outputImageMaxDimPixels_expected = 5000;

void renderDotToCPS(
    const core::Vector2& pos,
    double radius,
    CPS& output,
    const RGB& rgb )
{
    output.setColor( rgb );
    output.addCircle( pos, radius, false );
}

void renderStrokeToCPS( const core::model::Stroke& stroke, CPS& output, const RGB& rgb )
{
    if( stroke.zeroLength() ) {
        renderDotToCPS(
            stroke.curve().startPosition(),
            stroke.width( 0.0 ) * 0.5,
            output,
            rgb );
    } else {
        const auto widthFunctor = [ &stroke ]( double t )
        {
            return stroke.width( t );
        };

        const auto& posCurve = stroke.curve();

        /// Return the number of samples to take for a given C0 subcurve (possibly comprising multiple Bezier curves) of
        /// the stroke.
        const printCurves::SamplesPerInterval samplesPerInterval = [ & ]( const core::BoundingIntervald& t )
        {
            const auto posSubcurve = posCurve.extractCurveForTInterval( t.min(), t.max() );
            const auto numPosBeziers = posSubcurve->numBezierCurves( false );

            // More samples where the width curve is more complex.
            const Curve& widthCurve = stroke.widthCurve();
            const auto widthSubcurve = widthCurve.extractCurveForTInterval( t.min(), t.max() );
            const auto numWidthBeziers = widthSubcurve->numBezierCurves( false );
            size_t totalSamples = std::max< size_t >( numPosBeziers * 10, numWidthBeziers * 10 );

            // A legacy from me trying to get Pulaski to export fast enough.
            totalSamples = std::min< size_t >( 600, totalSamples );
            return totalSamples;
        };

        output.setColor( rgb );
        output.addVaryingWidthCurve( stroke.curve(), widthFunctor, samplesPerInterval );
    }
}

} // unnamed

void saveInputDrawingsEPS(
    const mashup::Drawings& drawings,
    const std::filesystem::path& path,
    const core::BoundingBoxd& canvasBounds )
{
    printCurves::CurvesPostScript cps( canvasBounds, canvasBounds.minDim() );
    for( int i = 0; i < mashup::DrawingID::NumDrawings; i++ ) {
        const RGB rgb = i == mashup::DrawingID::DrawingA
                ? RGB( 0.392, 0.008, 0.0353 )
                : RGB( 0., 0.020, 0.427 );

        const auto& drawing = drawings[ i ];
        drawing.forEach( [ & ]( const core::model::Stroke& stroke )
        {
            renderStrokeToCPS( stroke, cps, rgb );
        } );
    }
    printCurves::curvesPostScriptToEps( cps, path.u8string() );
}

void saveMashedUpDrawingEPS(
    const std::vector< std::unique_ptr< core::model::Stroke > >& strokes,
    const std::filesystem::path& path,
    const core::BoundingBoxd& canvasBounds )
{
    printCurves::CurvesPostScript cps( canvasBounds, canvasBounds.minDim() );
    const RGB rgb{ 0., 0., 0. };
    for( const auto& stroke : strokes ) {
        renderStrokeToCPS( *stroke, cps, rgb );
    }
    printCurves::curvesPostScriptToEps( cps, path.u8string() );
}

} // mashupDemo
