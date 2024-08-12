#include <view/consoleprogressbar.h>

namespace core {
namespace view {

ConsoleProgressBar::ConsoleProgressBar( std::ostream& stream, const std::string& leadup )
    : _stream( stream )
    , _leadup( leadup )
{
}

void ConsoleProgressBar::updateDisplay()
{
    // Refresh current line in console.
    _stream << '\r' << _leadup;
    if( _stages.size() ) {
        const auto& top = _stages.top();
        _stream << top.name;
        if( top.totalSteps > 0 ) {
            const auto percent = static_cast< int >(
                static_cast< double >( top.numStepsCompleted ) / static_cast< double >( top.totalSteps ) * 100. );
            _stream << " " << percent << "%";
        }
    }

    // Warning: This is not the proper way to handle overwriting all the previously printed
    // characters, but it works for MashupDemo's use of 'ConsoleProgressBar'. (Proper way
    // would involve calculating the _printed_ length of a string, without knowing what
    // kind of console we're printing in--without knowing the platform in other words.)
    const std::string moreBlankSpaces( 10, ' ' );
    _stream << moreBlankSpaces;

    _stream.flush();
}

} // view
} // core
