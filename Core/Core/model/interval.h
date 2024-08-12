#ifndef CORE_MODEL_INTERVAL_H
#define CORE_MODEL_INTERVAL_H

#include <array>

namespace core {
namespace model {

using Interval =  std::array< double, 2 >;

Interval fullInterval( bool forward );
double fromInterval( const Interval&, double f );
Interval reverseInterval( const Interval& );
/// If 'sampleFrom' is [0.5,0.6] and 'sample' is [1.0,0.5], return [0.6,0.55].
Interval remapInterval( const Interval& sampleFrom, const Interval& sample );

} // model
} // core

#endif // #include
