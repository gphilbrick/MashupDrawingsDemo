#ifndef CORE_STOPWATCH_H
#define CORE_STOPWATCH_H

//See http://stackoverflow.com/questions/6884093/warning-c4003-not-enough-actual-parameters-for-macro-max-visual-studio-2010
//Apparently, Windows.h introduces min/max macros which will screw up files that try to use std::min/max.  Even
//the std namespace qualifier is not enough to deal with this problem, apparently.

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif // #ifndef
#include <Windows.h>

namespace core {

class Stopwatch
{
public:
    /// Return the time elapsed in seconds since 'reset' was called, or since construction if
    /// 'reset' was never called.
    double updateSeconds();
    Stopwatch();
    void reset();
private:
    LARGE_INTEGER _baseTime;
    double _freq;
};

} // core

#else
#error Timer only defined for Windows
#endif

#endif // #include guard
