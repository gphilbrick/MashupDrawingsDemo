#include <model/interval.h>

#include <utility/boundinginterval.h>
#include <utility/mathutility.h>

namespace core {
namespace model {

Interval fullInterval( bool forward )
{
    return forward ? Interval{ 0.0, 1.0 } : Interval{ 1.0, 0.0 };
}

double fromInterval( const Interval& interval, double f )
{
    return mathUtility::lerp( interval[ 0 ], interval[ 1 ], f );
}

Interval reverseInterval( const Interval& interval )
{
    return { interval[ 1 ], interval[ 0 ] };
}

Interval remapInterval( const Interval& sampleFrom, const Interval& sample )
{
    return
    {
        mathUtility::lerp( sampleFrom[ 0 ], sampleFrom[ 1 ], sample[ 0 ] ),
        mathUtility::lerp( sampleFrom[ 0 ], sampleFrom[ 1 ], sample[ 1 ] )
    };
}

} // model
} // core
