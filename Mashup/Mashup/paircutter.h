#ifndef MASHUP_PAIRCUTTER_H
#define MASHUP_PAIRCUTTER_H

#include <Core/utility/mathutility.h>

#include <array>
#include <functional>
#include <vector>

namespace mashup {

/// Serves in situations where we need to repeatedly attempt a task involving
/// f_i in [0,1] and f_j in [0,1] (with both ranges discretized by 'StepsPerSide')
/// until the task succeeds, hopefully with high-as-possible f_i and f_j.
template< size_t StepsPerSide >
class PairCutter
{
public:
    /// Call 'task' on various f_i/f_j pairs (larger values before smaller)
    /// until 'task' returns true. If 'task' never returns true, return
    /// false; else return true.
    static bool doUntilSuccess( std::function< bool( double, double ) > task )
    {
        initPairsIfNeeded();

        for( const auto& pair : _pairs ) {
            if( task( pair[ 0 ], pair[ 1 ] ) ) {
                return true;
            }
        }
        return false;
    }
private:
    static void initPairsIfNeeded()
    {
        static_assert( StepsPerSide > 1, "StepsPerSide too low." );
        if( _pairs.size() > 0 ) {
            return;
        }

        for( size_t i = 0; i < StepsPerSide; i++ ) {
            const auto f_i = F_FROM_I( i, StepsPerSide );
            for( size_t j = 0; j < StepsPerSide; j++ ) {
                const auto f_j = F_FROM_I( j, StepsPerSide );
                _pairs.push_back( { f_i, f_j } );
            }
        }

        std::sort(
            _pairs.begin(),
            _pairs.end(),
            []( const FPair& a, const FPair& b )
            {
                // larger F values before smaller.
                return ( a[ 0 ] + a[ 1 ] ) > ( b[ 0 ] + b[ 1 ] );
            } );
    }

    /// f_i and f_j in [0,1]
    using FPair = std::array< double, 2 >;
    static inline std::vector< FPair > _pairs;
};

} // mashup

#endif // #include
