#ifndef CORE_CURVEINTERVAL_H
#define CORE_CURVEINTERVAL_H

namespace core {

/// A directed T-interval on a possibly-closed curve where T ranges from 0 to 1.
class CurveInterval
{
public:
    CurveInterval();
    CurveInterval( double tStart, double tEnd );
    /// If 'tIncreasing' is different from tStart<tEnd, that means this interval can
    /// only be used with a closed curve.
    CurveInterval( double tStart, double tEnd, bool tIncreasing );
    CurveInterval( const CurveInterval& );
    CurveInterval& operator=( const CurveInterval& );
    double tStart() const;
    double tEnd() const;
    bool tIncreasing() const;
    /// Return whether the interval goes tStart -> t=0 -> tEnd; in other words, return whether
    /// this interval can only be used with a closed curve.
    bool wrapsAround() const;
    /// f in [0,1]
    double t( double f ) const;
    double fFromT( double t ) const;
    /// In T
    double length() const;
private:
    double _tStart;
    double _tEnd;
    /// Disambiguates [tStart,tEnd] in cases where the curve is closed.
    bool _tIncreasing;
};

} // core

#endif // #include
