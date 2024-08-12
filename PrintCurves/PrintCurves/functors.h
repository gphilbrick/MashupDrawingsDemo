#ifndef PRINTCURVES_FUNCTORS_H
#define PRINTCURVES_FUNCTORS_H

#include <Core/utility/boundingbox.h>

#include <functional>

namespace printCurves {

/// Return the number of samples to take from the indicated T-interval on a varying-width curve ("stroke").
using SamplesPerInterval = std::function< size_t( const core::BoundingIntervald ) >;

/// For varying-width curves: this defines the width of a curve as a function of position in canvas space.
using CanvasPosToWidth = std::function< double( const core::Vector2& ) >;
/// For varying-width curves: this defines the width of a curve as a function of its parameter.
using ParamToWidth = std::function< double( double ) >;

/// This serves as a least common denominator of ParamToWidth and CanvasPosToWidth. The
/// parameters are 't', 'sample', and 'canvasPositions'. The understanding is
/// that either only 't' will be used (ParamToWidth) or everything except 't' will be used (CanvasPosToWidth).
using LowLevelWidthFunctor =
    std::function< double( double, const core::Vector2& ) >;

} // printCurves

#endif // #include
