#include "stopwatch.h"

namespace core {

Stopwatch::Stopwatch()
{
    reset();
}

void Stopwatch::reset()
{
    LARGE_INTEGER pf;
    QueryPerformanceFrequency(&pf);
    _freq = 1.0/((double)pf.QuadPart);
    QueryPerformanceCounter(&_baseTime);
    updateSeconds();
}

double Stopwatch::updateSeconds()
{
    LARGE_INTEGER val;
    QueryPerformanceCounter(&val);
    return ((double)(val.QuadPart-_baseTime.QuadPart))*_freq;
}

} // core
