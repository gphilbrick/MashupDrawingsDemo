#ifndef CORE_MODEL_CURVE_H
#define CORE_MODEL_CURVE_H

#include <memory>

namespace core {
class BSpline2;
} // core

namespace core {
namespace model {

using Curve = core::BSpline2;
using UniqueCurve = std::unique_ptr< Curve >;
using UniqueConstCurve = std::unique_ptr< const Curve >;

} // model
} // core

#endif // #include
