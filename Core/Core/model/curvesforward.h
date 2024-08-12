#ifndef CORE_MODEL_CURVES_H
#define CORE_MODEL_CURVES_H

// A separate forward-declaration oriented header file for involving vector.

#include <Core/model/curveforward.h>

#include <vector>

namespace core {
namespace model {

using UniqueCurves = std::vector< UniqueCurve >;
using UniqueConstCurves = std::vector< UniqueConstCurve >;
using RawCurves = std::vector< Curve* >;
using RawConstCurves = std::vector< const Curve* >;

} // model
} // core

#endif // #include
