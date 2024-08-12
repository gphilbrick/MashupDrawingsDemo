#ifndef PRINTCURVES_STROKEPROPERTIES_H
#define PRINTCURVES_STROKEPROPERTIES_H

#include <PrintCurves/cap.h>

namespace printCurves {

/// Data informing how to draw a "varying-width curve."
struct StrokeProperties
{
    StrokeProperties() : treatAsContinuous( false )
    {
    }

    Cap startCap;
    Cap endCap;

    /// This means that the curve being drawn should be considered to be smooth all the way, with no C0 corners.
    /// It disables breaking apart the curve apart at locations of degree n-1 knot multiplicity and handling the intervals
    /// separately.
    bool treatAsContinuous;
};

} // printCurves

#endif // #define
