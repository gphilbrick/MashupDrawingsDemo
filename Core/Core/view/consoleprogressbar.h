#ifndef CORE_VIEW_CONSOLEPROGRESSBAR_H
#define CORE_VIEW_CONSOLEPROGRESSBAR_H

#include <Core/view/progressbar.h>

#include <ostream>
#include <string>

namespace core {
namespace view {

/// A "progress bar" represented within a console via updating the same line.
class ConsoleProgressBar : public ProgressBar
{
public:
    /// Let 'leadup' be the start of the line that gets updated.
    ConsoleProgressBar( std::ostream& stream, const std::string& leadup );
protected:
    void updateDisplay() override;
private:
    std::ostream& _stream;
    const std::string _leadup;
};

} // view
} // core

#endif // #include
