#ifndef CORE_UTILITY_BUILDSPLINEEXCEPTION_H
#define CORE_UTILITY_BUILDSPLINEEXCEPTION_H

#include <stdexcept>

namespace core {

class BuildSplineException : public std::runtime_error
{
public:
    BuildSplineException( const char* msg ) : std::runtime_error( msg )
    {}
};

} // core

#endif // #include
