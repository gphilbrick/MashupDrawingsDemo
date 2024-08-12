#ifndef PRINTCURVES_MITEREDCURVE_H
#define PRINTCURVES_MITEREDCURVE_H

#include <PrintCurves/functors.h>

#include <vector>

namespace core {
class BSpline2;
class Vector2;
class Wall;
}

namespace printCurves {

/// Store in 'storeTimes' the T-values (other than 0 or 1) where the spline is only C0 continuous, i.e., the spline's corners. Store in
/// 'storeJoins' some "walls" representing the miter joints that would be used to locally put two segments of a stroke based on the spline
/// together.
void c0TimesAndMiteredJoins(
    const core::BSpline2& spline,
    std::vector< double >& storeTimes,
    std::vector< core::Wall >& storeJoins );

// ltwarning: KNOWN BUG: "BOX BUG"
//      (discovered while working on 'StrokeSegCollider' in blend-drawings)
//   If you very densely sample a box curve ('pos' is a closed box, 'widths' is long
//   relative to the width of the box-'Stroke' being represented), then the inner result
//   box ('storeLeft', say) will have some overextension "whiskers". These are pretty
//   intuitive in cause: they're almost like cusps except in the 1D context of not walking
//   fast enough outside of the one of the box-'Stroke''s corners.
//
//   Not sure if this really needs a fix and not willing to pursue one right now given how
//   heavily this is used in rendering code.
/// Store in 'storeLeft' and 'storeRight' equal-length polylines that can be used to "rib"-render
/// the polyline 'pos', given corresponding 'widths'. 'pos' and 'width's must have same size > 1.
/// 'storeLeft' and 'storeRight' will be the same length as 'pos' and 'widths'.
/// If 'pos' has equal endpoints, render it as if it were a closed curve.
void miteredOffsetSamples(
    const std::vector< core::Vector2 >& pos,
    const std::vector< double >& widths,
    std::vector< core::Vector2 >& storeLeft,
    std::vector< core::Vector2 >& storeRight,
    double maxRibFac = 10.0 );
void miteredOffsetSamples(
    const std::vector< core::Vector2 >& pos,
    const std::vector< double >& leftWidths,
    const std::vector< double >& rightWidths,
    std::vector< core::Vector2 >& storeLeft,
    std::vector< core::Vector2 >& storeRight,
    double maxRibFac = 10.0 );

/// Store in 'storeLeft' and 'storeRight' polylines--a pair of polylines for each segment of 'spline' (a segment being delineated by C0 "corners").
/// 'storeLeft' and 'storeRight' will have the same length. The i+1'th polyline in either 'storeLeft' or 'storeRight' starts out
/// at the same position where the i'th polyline before it ends. Use the functor parameters to determine the number of samples for
/// each section of 'spline' and the width of the "stroke" being rendered.
void offsetSamplesForMiteredJoinRender(
    const core::BSpline2& spline,
    std::vector< std::vector< core::Vector2 > >& storeLeft,
    std::vector< std::vector< core::Vector2 > >& storeRight,
    SamplesPerInterval samplesPerInterval,
    ParamToWidth tToWidth );
void offsetSamplesForMiteredJoinRender(
    const core::BSpline2& spline,
    std::vector< std::vector< core::Vector2 > >& storeLeft,
    std::vector< std::vector< core::Vector2 > >& storeRight,
    SamplesPerInterval samplesPerInterval,
    CanvasPosToWidth posToWidth );
void offsetSamplesForMiteredJoinRender(
    const core::BSpline2& spline,
    std::vector< std::vector< core::Vector2 > >& storeLeft,
    std::vector< std::vector< core::Vector2 > >& storeRight,
    SamplesPerInterval samplesPerInterval,
    LowLevelWidthFunctor widthFunctor );

} // printCurves

#endif // #include
