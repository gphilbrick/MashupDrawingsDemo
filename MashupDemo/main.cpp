#include <mashupinputs.h>
#include <prepareinputs.h>
#include <savetofile.h>

#include <Mashup/blenddrawings.h>
#include <Mashup/blendoptions.h>
#include <Mashup/drawings.h>
#include <Core/view/consoleprogressbar.h>

#include <iostream>
#include <filesystem>

const char* const epsExt = ".eps";

std::filesystem::path inputImagePath( const std::string& scenarioName )
{
    return { scenarioName + std::string{ "_inputs" } + std::string{ epsExt } };
}

std::filesystem::path outputImagePath( const std::string& scenarioName )
{
    return { scenarioName + std::string{ "_mashup" } + std::string{ epsExt } };
}

void runDemo( mashupDemo::MashupInputs&& inputs )
{
    static size_t numDemosRun = 0;

    std::cout << "\tScenario " << ( ++numDemosRun ) << ": " << inputs.name << std::endl;

    const auto saveInputsPath = inputImagePath( inputs.name );
    mashupDemo::saveInputDrawingsEPS( inputs.inputDrawings, saveInputsPath, inputs.canvasBounds );
    std::cout << "\t\tInput drawings viewable in " << saveInputsPath << std::endl;

    // Mash up the input drawings.
    std::cout << "\t\tBeginning mashup process..." << std::endl;
    core::view::ConsoleProgressBar showProg( std::cout, "\t\t" );
    mashup::BlendDrawings createMashup( std::move( inputs.inputDrawings ), inputs.options, &showProg );
    createMashup.perform();
    std::cout << "\r\t\tMashup complete." << std::endl;

    const auto saveOutputPath = outputImagePath( inputs.name );
    mashupDemo::saveMashedUpDrawingEPS( createMashup.result(), saveOutputPath, inputs.canvasBounds );
    std::cout << "\t\tMashup viewable in " << saveOutputPath << std::endl;
}

int main( int, char *[] ) 
{
    std::cout << std::endl << "Drawing Mashup Demo:" << std::endl;

    // This shows a canonical mashup where strokes dominate other-drawing strokes
    // based on stroke width.
    runDemo( mashupDemo::prepareScenario_basic() );
    // Preserve-drawing mode: Drawing A gets preserved in the result.
    runDemo( mashupDemo::prepareScenario_preserveA() );
    // An example where an AB field is used to determine when one drawing yields to the other.
    runDemo( mashupDemo::prepareScenario_useABField() );

    std::cout << "Demo complete." << std::endl;
    return 0;
}
