#ifndef CORE_EXCEPTIONS_RUNTIMEERROR_H
#define CORE_EXCEPTIONS_RUNTIMEERROR_H

#include <stdexcept>
#include <string>

namespace core {
namespace exceptions {

// https://stackoverflow.com/questions/348833/how-to-know-the-exact-line-of-code-where-an-exception-has-been-caused
class RuntimeError : public std::runtime_error
{
public:
    RuntimeError( const char* msg, const char* file, int line );
    const char* what() const noexcept override;
private:
    std::string _msg;
};

#define THROW_RUNTIME( stringLiteral ) throw ::core::exceptions::RuntimeError( stringLiteral, __FILE__, __LINE__ );
#define THROW_UNEXPECTED THROW_RUNTIME( "Unexpected" )

}
}

#endif // #include
