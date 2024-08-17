#include <exceptions/runtimeerror.h>

#include <sstream>

namespace core {
namespace exceptions {

RuntimeError::RuntimeError( const char* msg, const char* file, int line )
    : std::runtime_error( msg )
{
    std::stringstream ss;
    ss << "'" << msg << "' at line " << line << " of " << file;
    _msg = ss.str();
}

const char* RuntimeError::what() const noexcept
{
    return _msg.c_str();
}

} // exceptions
} // core
