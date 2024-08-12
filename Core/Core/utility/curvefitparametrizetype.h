#ifndef CORE_CURVEFITPARAMETRIZETYPE_H
#define CORE_CURVEFITPARAMETRIZETYPE_H

namespace core {

/// For least-squares curve fitting.
enum CurveFitParametrizeType
{
    SplitIntervalEvenly,
    ChordLength,
    UseXAsT
};

} // core

#endif // #include
