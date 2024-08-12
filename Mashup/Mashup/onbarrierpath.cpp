#include <onbarrierpath.h>

#include <Core/exceptions/runtimeerror.h>

namespace mashup {

using Polyline= core::model::Polyline;
using Pos = core::model::Pos;

size_t OnBarrierPath::length() const
{
    return pos.size();
}

void OnBarrierPath::clear()
{
    pos.clear();
    normal.clear();
    closed = false;
}

core::model::Pos OnBarrierPath::dir( size_t i ) const
{
    if( i == pos.size() - 1 ) {
        return pos[ i ] - pos[ i - 1 ];
    } else {
        return pos[ i + 1 ] - pos[ i ];
    }
}

/// Extract a polyline from 'obp' starting at 'startIdx', moving up or down index-wise as per 'increaseIdx', and ending
///     (1) upon reaching the end of 'obp' if 'obp' is not closed
///     (2) upon making a circuit of 'obp' if 'obp' is closed
///     (3) upon 'lastSeg' returning true (and setting the last position in 'ret' in its third parameter).
Polyline OnBarrierPath::extractPolyline(
                     size_t startIdx,
                     bool increaseIdx,
                     std::function< bool( const Pos&, const Pos&, Pos& ) > lastSeg ) const
{
    const auto len = length();
    if( len < 2 ) {
        THROW_UNEXPECTED;
    }

    Polyline ret{ pos[ startIdx ] };

    size_t numSegs = 0;
    {
        if( closed ) {
            numSegs = len - 1;
        } else {
            numSegs = increaseIdx
                          ? len - 1 - startIdx
                          : startIdx;
        }
    }

    size_t curIdx = startIdx;
    for( size_t segIdx = 0; segIdx < numSegs; segIdx++ ) {
        size_t nextIdx = curIdx;
        if( increaseIdx ) {
            nextIdx = ( curIdx + 1 ) % len;
        } else {
            if( nextIdx == 0 ) {
                nextIdx = len - 1;
            } else {
                nextIdx--;
            }
        }

        const auto& polyA = pos[ curIdx ];
        const auto& polyB = pos[ nextIdx ];

        Pos storeLastP;
        const auto isLast = lastSeg( polyA, polyB, storeLastP );
        if( isLast ) {
            ret.push_back( storeLastP );
            break;
        } else {
            ret.push_back( polyB );
        }

        curIdx = nextIdx;
    }

    return ret;
}

} // mashup
