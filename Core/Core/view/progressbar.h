#ifndef CORE_VIEW_PROGRESSBAR_H
#define CORE_VIEW_PROGRESSBAR_H

#include <stack>
#include <string>

namespace core {

namespace model {
class CanvasLayer;
} // model

namespace view {

class ProgressBar
{
public:
    ProgressBar();
    virtual ~ProgressBar();
    /// Clear the stage stack and push a new one w/ 'name' and 'numSteps'.
    void startOnlyStage( const std::string& name, int numSteps = defaultNumSteps );
    /// 'numSteps' must be greater than 0.
    void pushStage( const std::string& nameOfStage, int numSteps = defaultNumSteps );
    void popStage();
    size_t stageStackSize() const;
    /// Change the displayed progress for the current stage. 'numStepsCompleted' should be no larger
    /// than the corresponding stage's creation 'numSteps'.
    void update( int numStepsCompleted );
    /// 'f' in [0,1].
    void updateF( double f );

    // This is meant to make dialog-using 'ProgressBar's not waste so much time in UI updating.
    /// 'f' in [0,1).
    /// Do not update the display on 'update' unless the amount of progress completed is
    /// at least 'f' times the number of steps to complete for current stage have been
    /// completed since display was last updated.
    ///
    /// If 'f' is zero, update display on all 'update' calls.
    void setIgnoreInterval( double f );

    static constexpr int defaultNumSteps = 100;
protected:
    virtual void updateDisplay() = 0;
    struct Stage
    {
        std::string name;
        int totalSteps = 0;
        int numStepsCompleted = 0;
        int numStepsCompleted_lastDisplayed = 0;
    };
    std::stack< Stage > _stages;
private:
    void updateDisplayWrapper();
    double _ignoreInterval;
};

} // view
} // core

#endif // #include
