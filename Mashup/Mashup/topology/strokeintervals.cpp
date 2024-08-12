#ifndef MASHUP_STROKEINTERVAL_H
#define MASHUP_STROKEINTERVAL_H

#include <topology/strokeintervals.h>

#include <substroke.h>

#include <Core/exceptions/runtimeerror.h>

#include <algorithm>
#include <iterator>

namespace mashup {
namespace topology {

namespace {

/// Throw an exception if 'occ' is invalid input to a 'StrokeIntervals'.
void validate( const TIntervals& occ )
{
    if( occ.size() == 0 ) {
        return;
    }

    for( size_t i = 0; i < occ.size(); i++ ) {
        const auto& a = occ[ i ];
        if( a.min() < 0. || a.min() > 1. || a.max() < 0. || a.max() > 1. ) {
            THROW_RUNTIME( "Invalid 'occ'" );
        }

        if( i < occ.size() - 1 ) {
            const auto& b = occ[ i + 1 ];
            if( b.min() <= a.max() ) {
                THROW_RUNTIME( "Occluded intervals must not touch or overlap" );
            }
        }
    }
}

} // unnamed

StrokeIntervals::StrokeIntervals()
    : StrokeIntervals( TIntervals{ TInterval{ 0., 1. } } )
{
}

StrokeIntervals::StrokeIntervals( const TIntervals& unoccludedDirty )
{
    TIntervals unoccluded;
    std::copy_if( unoccludedDirty.begin(),
                  unoccludedDirty.end(),
                  std::back_inserter( unoccluded ),
                  []( const TInterval& i )
                  {
                     return !i.zeroLength();
                  } );
    validate( unoccluded );
    _numUnoccluded = unoccluded.size();

    if( unoccluded.size() == 0 ) {
        _occludedAtZero = true;
        return;
    }
    _occludedAtZero = unoccluded.front().min() > 0.;

    if( _occludedAtZero ) {
        _splitT.push_back( 0. );
    }
    for( const auto& i : unoccluded ) {
        _splitT.push_back( i.min() );
        _splitT.push_back( i.max() );
    }
    if( _splitT.back() < 1. ) {
        _splitT.push_back( 1. );
    }

    const size_t totalIntervals = _splitT.size() - 1;
    if( _numUnoccluded > totalIntervals ) {
        THROW_UNEXPECTED;
    }
    _numOccluded = totalIntervals -_numUnoccluded;
}

TIntervals StrokeIntervals::intervals( bool occluded ) const
{
    TIntervals ret;
    size_t idx = ( occluded == _occludedAtZero ) ? 0 : 1;
    while( idx < _splitT.size() - 1 ) {
        ret.push_back( { _splitT[ idx ], _splitT[ idx + 1 ] } );
        idx += 2;
    }
    return ret;
}

bool StrokeIntervals::anyUnoccluded() const
{
    return !_occludedAtZero || _splitT.size() > 2;
}

void StrokeIntervals::unoccludedAdjacentTo( size_t occludedIdx,
                           boost::optional< size_t >& storeBefore,
                           boost::optional< size_t >& storeAfter ) const
{
    storeBefore = boost::none;
    storeAfter = boost::none;

    if( _occludedAtZero ) {
        if( occludedIdx > 0 ) {
            storeBefore = occludedIdx - 1;
        }
        if( occludedIdx <= _numUnoccluded - 1 ) {
            storeAfter =  occludedIdx;
        }
    } else {
        storeBefore = occludedIdx;
        if( occludedIdx + 1 <= _numUnoccluded - 1 ) {
            storeAfter = occludedIdx + 1;
        }
    }
}

boost::optional< size_t > StrokeIntervals::crossingIndex( const Substroke& ss ) const
{
    // Find the index of the first unoccluded interval that contains mid-T of 'ss'.
    const auto tMid = ( ss.t[ 0 ] + ss.t[ 1 ] ) / 2.;

    const auto unocc = intervals( false );
    for( size_t unoccIdx = 0; unoccIdx < unocc.size(); unoccIdx++ ) {
        const auto& unoccInterval = unocc[ unoccIdx ];
        if( unoccInterval.contains( tMid ) ) {
            int crossingIdx = 0;
            if( ss.tIncreasing() ) {
                // Return idx of crossing after 'unoccInterval'.
                crossingIdx = _occludedAtZero ? static_cast< int >( unoccIdx ) + 1 : static_cast< int >( unoccIdx );
            } else {
                // Return idx of crossing before 'unoccInterval'.
                crossingIdx = _occludedAtZero ? static_cast< int >( unoccIdx ) : static_cast< int >( unoccIdx ) - 1;
            }

            if( crossingIdx > -1 && crossingIdx < static_cast< int >( _numOccluded )  ) {
                return crossingIdx;
            }
            break;
        }
    }

    return boost::none;
}

} // topology
} // mashup

#endif // #include
