#include <view/progressbar.h>

#include <exceptions/runtimeerror.h>

namespace core {
namespace view {

ProgressBar::ProgressBar()
    : _ignoreInterval( 0. )
{}

ProgressBar::~ProgressBar()
{}

void ProgressBar::startOnlyStage( const std::string& name, int numSteps )
{
    _stages = {};
    pushStage( name, numSteps );
}

void ProgressBar::pushStage( const std::string& nameOfStage, int numSteps )
{
    Stage stage;
    stage.name = nameOfStage;
    stage.totalSteps = numSteps;
    _stages.push( stage );
    updateDisplayWrapper();
}

void ProgressBar::popStage()
{
    if( _stages.size() > 1 ) {
        _stages.pop();
        updateDisplayWrapper();
    }
}

void ProgressBar::updateF( double f )
{
    const int numSteps = _stages.top().totalSteps;
    update( static_cast< int >( f * static_cast< double >( numSteps ) ) );
}

void ProgressBar::update( int numStepsCompleted )
{
    if( _stages.size() ) {
        auto& stage = _stages.top();
        stage.numStepsCompleted = numStepsCompleted;

        const auto intervalSinceLastDisplay =
            stage.totalSteps > 0
            ? static_cast< double >( stage.numStepsCompleted - stage.numStepsCompleted_lastDisplayed )
                                                        / static_cast< double >( stage.totalSteps )
            : 1.;
        if( intervalSinceLastDisplay >= _ignoreInterval ) {
            updateDisplayWrapper();
        }
    } else {
        THROW_RUNTIME( "Need to start a stage first" );
    }
}

size_t ProgressBar::stageStackSize() const
{
    return _stages.size();
}

void ProgressBar::updateDisplayWrapper()
{
    _stages.top().numStepsCompleted_lastDisplayed = _stages.top().numStepsCompleted;
    updateDisplay();
}

void ProgressBar::setIgnoreInterval( double f )
{
    _ignoreInterval = f;
}

} // view
} // core
