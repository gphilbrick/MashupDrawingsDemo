#include <strokesegcollider.h>

#include <drawings.h>
#include <onbarrierpath.h>
#include <strokeback.h>
#include <strokepoly.h>
#include <substroke.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/model/lineback.h>
#include <Core/model/stroketools.h>

#include <Core/utility/boundinginterval.h>
#include <Core/utility/mathutility.h>

#include <algorithm>
#include <set>

namespace mashup {

using Hit = StrokeSegCollider::Hit;
using Pos = core::model::Pos;
using AB = core::model::Seg;
using Polyline= core::model::Polyline;
using SegID = StrokeSegColliderMetadata::SegID;

StrokeSegCollider::StrokeSegCollider( const core::model::BoundingBox& canvasBounds )
    : Base( canvasBounds, 100 )
    , _nextSegID( 0 )
{
}

void StrokeSegCollider::removeStroke( StrokeHandle stroke )
{
    for( const auto& coords : _strokeToInvolvedCoords[ stroke ] ) {
        auto& bin = _grid.getRef( coords );
        CellContents filtered;
        for( const auto& pair : bin ) {
            if( !( pair.metadata.stroke == stroke ) ) {
                filtered.push_back( pair );
            }
        }
        bin = filtered;
    }
    _strokeToInvolvedCoords[ stroke ].clear();

    // _idToSWD
    {
        decltype( _idToSWD ) backup;
        // https://www.techiedelight.com/filter-a-map-in-cpp/
        std::copy_if(
            _idToSWD.begin(),
            _idToSWD.end(),
            std::inserter( backup, backup.begin() ),
            [ &stroke ]( const std::pair< SegID, SegWithData >& pair )
            {
                return pair.second.metadata.stroke != stroke;
            } );
        _idToSWD = std::move( backup );
    }
}

std::vector< StrokeSegCollider::SegWithData > StrokeSegCollider::strokeSegsWithinRange( const IPos& xy, double range ) const
{
    const auto cx = xy.x();
    const auto cy = xy.y();

    const int nw = static_cast< int >( neighborhoodWidth( range ) );
    const int halfNW = nw / 2;

    std::set< SegID > segIds;
    std::vector< SegWithData > ret;
    for( int x = cx - halfNW; x <= cx + halfNW; x++ ) {
        for( int y = cy - halfNW; y <= cy + halfNW; y++ ) {
            if( _grid.isValidCoord( x, y ) ) {
                const auto& bin = _grid.getRef( x, y );
                for( const auto& pair : bin ) {
                    if( segIds.find( pair.metadata.segID ) != segIds.end() ) {
                        continue;
                    } else {
                        segIds.emplace( pair.metadata.segID );
                    }

                    SegWithData seg;
                    seg.seg = pair.seg;
                    seg.metadata = pair.metadata;
                    ret.push_back( seg );
                }
            }
        }
    }
    return ret;
}

std::vector< StrokeSegCollider::SegWithData > StrokeSegCollider::strokeSegsWithinRange( const Pos& posCanvas, double range ) const
{
    return strokeSegsWithinRange( cellCoords( posCanvas ), range );
}

void StrokeSegCollider::addStroke( const StrokePoly& sPoly )
{
    if( !sPoly.participates() ) {
        return;
    }

    const auto* const stroke = sPoly.stroke;
    auto& involvedCoords = _strokeToInvolvedCoords[ stroke ];

    const auto addSWD = [ & ]( const SegWithData& swd )
    {
        addSeg( swd.seg, swd.metadata, &involvedCoords );
        _idToSWD[ swd.metadata.segID ] = swd;
    };

    // The two offset curves of the 'Stroke'
    const auto& t = sPoly.t;
    for( size_t i = 0; i < NumSides; i++ ) {
        const auto& side = sPoly.sides[ i ];
        const auto& sideNorms = sPoly.sideNormals[ i ];
        const auto numSegs = sideNorms.size();
        SegsWithData swd( numSegs );

        for( size_t i = 0; i < numSegs; i++ ) {
            const auto tA = t[ i ];
            const auto tB = t[ i + 1 ];
            swd[ i ].seg = { side[ i ], side[ i + 1 ] };

            auto& meta = swd[ i ].metadata;
            meta.normal = sideNorms[ i ];
            meta.t = { tA, tB };
            meta.stroke = stroke;
            meta.segID = _nextSegID++;
        }
        // break 'SegID' sequence (for debugging, not necessary)
        _nextSegID++;

        // make inter-seg connections.
        if( numSegs > 1 ) {
            for( size_t i = 0; i < numSegs - 1; i++ ) {
                auto& swdA = swd[ i ];
                auto& swdB = swd[ i + 1 ];
                swdA.metadata.next = swdB.metadata.segID;
                swdB.metadata.prev = swdA.metadata.segID;
            }
            if( sPoly.closed() ) {
                auto& last = swd.back();
                auto& first = swd.front();
                last.metadata.next = first.metadata.segID;
                first.metadata.prev = last.metadata.segID;
            }
        }

        for( const auto& toAdd : swd ) {
            addSWD( toAdd );
        }
    }

    // Start/end caps
    if( !sPoly.closed() ) {
        SegWithData startCap;
        startCap.seg = AB{ sPoly.sides[ Left ].front(), sPoly.sides[ Right ].front() };
        startCap.metadata.normal = *sPoly.capNormal_T0;
        startCap.metadata.t = { 0., 0. };
        startCap.metadata.stroke = stroke;
        startCap.metadata.isCap = true;
        startCap.metadata.segID = _nextSegID++;
        // break 'SegID' sequence (for debugging, not necessary)
        _nextSegID++;
        addSWD( startCap );

        SegWithData endCap;
        endCap.seg = AB{ sPoly.sides[ Left ].back(), sPoly.sides[ Right ].back() };
        endCap.metadata.normal = *sPoly.capNormal_T1;
        endCap.metadata.t = { 1., 1. };
        endCap.metadata.stroke = stroke;
        endCap.metadata.isCap = true;
        endCap.metadata.segID = _nextSegID++;
        // break 'SegID' sequence (for debugging, not necessary)
        _nextSegID++;
        addSWD( endCap );
    }
}

bool StrokeSegCollider::hitsAnything( const core::model::Polyline& hitter ) const
{
    if( hitter.size() < 2 ) {
        return false;
    }

    for( size_t i = 0; i < hitter.size() - 1; i++ ) {
        std::set< SegID > seenSegs;
        const AB segHitter{ hitter[ i ], hitter[ i + 1 ] };
        const auto coordsToCheck = checkCoords( segHitter );
        for( const auto& coord : coordsToCheck ) {
            if( _grid.isValidCoord( coord ) ) {
                const auto& bin = _grid.getRef( coord );
                for( const auto& swd : bin ) {
                    if( seenSegs.find( swd.metadata.segID ) != seenSegs.end() ) {
                        continue;
                    }
                    seenSegs.emplace( swd.metadata.segID );
                    const auto& segSubstroke = swd.seg;
                    Pos hit;
                    if( core::mathUtility::segmentsIntersect( segHitter, segSubstroke, hit ) ) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool StrokeSegCollider::hitsAnythingPassing( const core::model::Polyline& hitter,
                         std::function< bool( const SegWithData& ) > testSWD ) const
{
    if( hitter.size() < 2 ) {
        return false;
    }

    for( size_t i = 0; i < hitter.size() - 1; i++ ) {
        std::set< SegID > seenSegs;
        const AB segHitter{ hitter[ i ], hitter[ i + 1 ] };
        const auto coordsToCheck = checkCoords( segHitter );
        for( const auto& coord : coordsToCheck ) {
            if( _grid.isValidCoord( coord ) ) {
                const auto& bin = _grid.getRef( coord );
                for( const auto& swd : bin ) {
                    if( seenSegs.find( swd.metadata.segID ) != seenSegs.end() ) {
                        continue;
                    }
                    seenSegs.emplace( swd.metadata.segID );

                    if( !testSWD( swd ) ) {
                        continue;
                    }

                    const auto& segSubstroke = swd.seg;
                    Pos hit;
                    if( core::mathUtility::segmentsIntersect( segHitter, segSubstroke, hit ) ) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool StrokeSegCollider::hitsAnythingPassing( const core::model::Polyline& hitter,
                                            std::function< bool( StrokeHandle, double ) > testStrokeAndT ) const
{
    if( hitter.size() < 2 ) {
        return false;
    }

    for( size_t i = 0; i < hitter.size() - 1; i++ ) {
        std::set< SegID > seenSegs;
        const AB segHitter{ hitter[ i ], hitter[ i + 1 ] };
        const auto coordsToCheck = checkCoords( segHitter );
        for( const auto& coord : coordsToCheck ) {
            if( _grid.isValidCoord( coord ) ) {
                const auto& bin = _grid.getRef( coord );
                for( const auto& swd : bin ) {
                    if( seenSegs.find( swd.metadata.segID ) != seenSegs.end() ) {
                        continue;
                    }
                    seenSegs.emplace( swd.metadata.segID );

                    const auto& segSubstroke = swd.seg;
                    Pos hit;
                    if( core::mathUtility::segmentsIntersect( segHitter, segSubstroke, hit ) ) {
                        const auto f = segSubstroke.t( hit );
                        const auto tStroke = core::mathUtility::lerp( swd.metadata.t[ 0 ], swd.metadata.t[ 1 ], f );
                        const auto strokeHandle = swd.metadata.stroke;
                        if( testStrokeAndT( strokeHandle, tStroke ) ) {
                            return true;
                        } else {
                            continue;
                        }
                    }
                }
            }
        }
    }
    return false;
}

void StrokeSegCollider::sameDrawingHits( DrawingToSameDrawingHits& store, const Drawings& d ) const
{
    // Clear them all.
    for( int dID = 0; dID < DrawingID::NumDrawings; dID++ ) {
        auto& sdh = store[ dID ];
        sdh.clear();
    }

    // Prevent checking same segments against each other twice.
    using SegIDPair = std::array< Metadata::SegID, 2 >;
    std::set< SegIDPair > seenPairs;

    // Scan all cells
    _grid.forEveryPos(
    [ & ]( const CellContents& bin )
    {
        const auto numSegs = bin.size();
        for( size_t i = 0; i < numSegs; i++ ) {
            const auto& seg_i = bin[ i ].seg;
            const auto stroke_i = bin[ i ].metadata.stroke;
            const auto drawing_i = d.whichDrawing( stroke_i );
            const auto segID_i = bin[ i ].metadata.segID;

            for( size_t j = i + 1; j < numSegs; j++ ) {
                const auto& seg_j = bin[ j ].seg;
                const auto stroke_j = bin[ j ].metadata.stroke;
                const auto drawing_j = d.whichDrawing( stroke_j );
                const auto segID_j = bin[ j ].metadata.segID;
                
                if( drawing_i != drawing_j || drawing_i == DrawingID::NumDrawings ) {
                    continue;
                }

                // Prevent unnecessary work
                const SegIDPair pair{ segID_i, segID_j };
                if( seenPairs.find( pair ) != seenPairs.end() ) {
                    continue;
                }
                seenPairs.emplace( pair );

                core::model::Pos hit;
                if( core::mathUtility::segmentsIntersect( seg_i, seg_j, hit ) ) {
                    const auto& tRange_strokeI = bin[ i ].metadata.t;
                    const auto& tRange_strokeJ = bin[ j ].metadata.t;
                    const auto t_strokeI = core::mathUtility::lerp(
                        tRange_strokeI[ 0 ],
                        tRange_strokeI[ 1 ],
                        seg_i.t( hit ) );
                    const auto t_strokeJ = core::mathUtility::lerp(
                        tRange_strokeJ[ 0 ],
                        tRange_strokeJ[ 1 ],
                        seg_j.t( hit ) );

                    // If this is a same-stroke intersection, there is a risk that we're actually
                    // detecting a cusp in one of the stroke's sides. Crude way to reduce these
                    // false positives is to require a
                    const auto minTGapForSameStroke = 0.1;

                    if( stroke_i != stroke_j || std::abs( t_strokeI - t_strokeJ ) >= minTGapForSameStroke ) {
                        store[ drawing_i ].addHit( stroke_i, stroke_j, t_strokeI, t_strokeJ );
                    }
                }
            }
        }
    } );
}

boost::optional< Hit > StrokeSegCollider::firstHit( const AB& ab, SWDPredicate pred, bool ignoreFromBehind ) const
{
    const auto coordsToCheck = checkCoords( ab );

    double shortestDist = std::numeric_limits< double >::max();
    boost::optional< Hit > ret;

    const auto abLength = ab.length();

    std::set< SegID > seenSegs;

    for( const auto& coord : coordsToCheck ) {
        if( _grid.isValidCoord( coord ) ) {
            const auto& bin = _grid.getRef( coord );
            for( const auto& swd : bin ) {
                if( seenSegs.find( swd.metadata.segID ) == seenSegs.cend() ) {
                    seenSegs.emplace( swd.metadata.segID );
                } else {
                    continue;
                }

                // Filter by 'pred'.
                if( pred && !pred( swd ) ) {
                    continue;
                }

                // Ignore backwards hits.
                if( ignoreFromBehind ) {
                    const auto& barrNorm = swd.metadata.normal;
                    if( Pos::dot( barrNorm, ab.asVec() ) > 0. ) {
                        continue;
                    }
                }

                const auto& seg = swd.seg;
                Pos hitPos;
                if( core::mathUtility::segmentsIntersect( ab, seg, hitPos ) ) {
                    const auto distTo = ( hitPos - ab.a ).length();
                    if( distTo < shortestDist ) {
                        shortestDist = distTo;
                        const auto f_ab = std::clamp( distTo / abLength, 0., 1. );
                        const auto f_seg = std::clamp( ( hitPos - seg.a ).length() / seg.length(), 0., 1. );
                        Hit hit;
                        hit.fHitter = f_ab;
                        hit.swd = swd;
                        hit.strokeT = core::mathUtility::lerp( swd.metadata.t[ 0 ], swd.metadata.t[ 1 ], f_seg );
                        hit.pos = hitPos;
                        ret = hit;
                    }
                }
            }
        }
    }

    return ret;
}

std::vector< Hit > StrokeSegCollider::allHits( const AB& ab, SWDPredicate pred, bool ignoreFromBehind ) const
{
    const auto coordsToCheck = checkCoords( ab );

    std::vector< Hit > ret;

    const auto abLength = ab.length();

    std::set< SegID > seenSegs;

    for( const auto& coord : coordsToCheck ) {
        if( _grid.isValidCoord( coord ) ) {
            const auto& bin = _grid.getRef( coord );
            for( const auto& swd : bin ) {
                if( seenSegs.find( swd.metadata.segID ) == seenSegs.cend() ) {
                    seenSegs.emplace( swd.metadata.segID );
                } else {
                    continue;
                }

                // Filter by 'pred'.
                if( pred && !pred( swd ) ) {
                    continue;
                }

                // Ignore backwards hits.
                if( ignoreFromBehind ) {
                    const auto& barrNorm = swd.metadata.normal;
                    if( Pos::dot( barrNorm, ab.asVec() ) > 0. ) {
                        continue;
                    }
                }

                const auto& seg = swd.seg;
                Pos hitPos;
                if( core::mathUtility::segmentsIntersect( ab, seg, hitPos ) ) {
                    const auto distTo = ( hitPos - ab.a ).length();
                    const auto f_ab = std::clamp( distTo / abLength, 0., 1. );
                    const auto f_seg = std::clamp( ( hitPos - seg.a ).length() / seg.length(), 0., 1. );
                    Hit hit;
                    hit.fHitter = f_ab;
                    hit.swd = swd;
                    hit.strokeT = core::mathUtility::lerp( swd.metadata.t[ 0 ], swd.metadata.t[ 1 ], f_seg );
                    hit.pos = hitPos;
                    ret.push_back( hit );
                }
            }
        }
    }

    return ret;
}

OnBarrierPath StrokeSegCollider::onBarrierPath(
    const AB& hitter, bool bothDirs, size_t& storeStartIndex ) const
{
    storeStartIndex = 0;
    const auto hit = firstHit( hitter, nullptr, true );
    if( !hit ) {
        return {};
    }
    const bool goWithBarr = Pos::dot( hitter.asVec(), hit->swd.seg.asVec() ) > 0.;
    auto firstHalf = onBarrierPath( *hit, goWithBarr );
    if( !bothDirs || firstHalf.length() == 0 || firstHalf.closed ) {
        // Nothing more to do.
        return firstHalf;
    }

    // Find the second half.
    auto secondHalf = onBarrierPath( *hit, !goWithBarr );
    if( secondHalf.length() == 0 ) {
        return firstHalf;
    }
    if( secondHalf.closed ) {
        // ltwarning: Proceeding in spite of bug
        //
        // In principle, this should be an error (why didn't 'firstHalf' pick up this loop?)
        // but if we're accepting that 'onBarrierPath' will not always work right, we need
        // to have a graceful fallback in this situation. Pretend that 'secondHalf' _is_
        // 'firstHalf', and reverse it to achieve that effect.
        OnBarrierPath rev;
        rev.closed = true;
        rev.pos = { secondHalf.pos.front() };
        rev.normal = { secondHalf.normal.front() };
        if( secondHalf.length() < 2 ) {
            THROW_UNEXPECTED;
        }
        for( size_t i = secondHalf.pos.size() - 1; i > 0; i-- ) {
            rev.pos.push_back( secondHalf.pos[ i ] );
            rev.normal.push_back( secondHalf.normal[ i ] );
        }
        return rev;
    }

    // Stitch them together (there's probably a faster way to do this).
    storeStartIndex = secondHalf.length() - 1;

    std::reverse( secondHalf.pos.begin(), secondHalf.pos.end() );
    std::reverse( secondHalf.normal.begin(), secondHalf.normal.end() );

    secondHalf.pos.insert( secondHalf.pos.end(), firstHalf.pos.begin() + 1, firstHalf.pos.end() );
    secondHalf.normal.insert( secondHalf.normal.end(), firstHalf.normal.begin() + 1, firstHalf.normal.end() );
    return secondHalf;
}

OnBarrierPath StrokeSegCollider::onBarrierPath( const Hit& start, bool goWithBarr ) const
{
    // KNOWN BUG: 'seenSegs'
    // In principle, a legal path can visit the same segment 'seg' twice or many more times, but
    // this algorithm will terminate if it ever sees 'seg' more than once. See 'seenSegsBug.cvs'.
    //
    // Solution would be (1) actually use a robust topological structure a la CGAL for this business,
    // or (2) use a more sophisticated, i.e., brittle mechanism for infinite loop prevention.

    OnBarrierPath ret;
    ret.pos = { start.pos };
    ret.normal = { start.swd.metadata.normal };

    // These two track our "on-barrier" position.
    SegID curSegID = start.swd.metadata.segID;
    std::set< SegID > seenSegs{ curSegID }; // prevent inf loop
    SegWithData curSWD = start.swd;

    while( true ) {
        // Handle movement along an on-barrier path.
        boost::optional< SegID > nextSegID;
        const SegWithData* nextSWD = nullptr;
        auto nextBarrierPos = goWithBarr ? curSWD.seg.b : curSWD.seg.a;
        // See if we get interrupted while trying to move from 'curPos'
        // to 'nextPos'. Ignore 'curSegID' or adjacent segs.
        const auto& prevBarrierPos = ret.pos.back();

        // Find out if we're interrupted by anything while moving from 'prevBarrierPos'
        // to 'nextBarrierPos' along segment 'nextSegID'. Use 'hitsAllowed' to filter
        // out interruptions that we want to ignore.
        const auto hitsAllowed = [ & ]( const SegWithData& swd )
        {
            // Ignore segments that are adjacent to 'curSegID' on the
            // same 'Stroke'-side.
            const auto encounteredID = swd.metadata.segID;
            if( curSWD.metadata.next == encounteredID ||
                curSWD.metadata.prev == encounteredID ) {
                return false;
            }

            // If this is a same-'Stroke' interruption and the 'Stroke' is closed, just
            // ignore it. This is a Band-Aid for dealing with the buggy redundant segments
            // I'm getting sometimes when closed 'Stroke's are polyline-approximated using
            // mitered joints. Cost of this is that if a closed 'Stroke' actually does
            // intersect itself (like a figure-eight) then that crossing won't be detected.
            const auto* const curSegStroke = curSWD.metadata.stroke;
            if( curSegStroke == swd.metadata.stroke
                && curSegStroke->closed() ) {
                return false;
            }

            // Ignore 'Stroke'-cap segments.
            if( swd.metadata.isCap ) {
                return false;
            }

            return true;
        };
        const auto hit = firstHit( AB{ prevBarrierPos, nextBarrierPos }, hitsAllowed, true );

        if( hit ) {
            const auto interruptingSegID = hit->swd.metadata.segID;
            if( seenSegs.find( interruptingSegID ) == seenSegs.end() ) {
                seenSegs.emplace( hit->swd.metadata.segID );
                nextSegID = interruptingSegID;
                nextSWD = &hit->swd;

                // Decide whether to go with or against the T-grain of the
                // new 'Stroke' side we've run into.
                const auto& curNormal = curSWD.metadata.normal;
                const auto barrDir = nextSWD->seg.asVec();
                goWithBarr = Pos::dot( curNormal, barrDir ) > 0.;

                // Abort on encountering "vertex-sharer":
                // Because 'StrokeSegCollider' isn't very advanced, this algorithm breaks down in situations
                // where we are moving from A->B and get interrupted almost exactly at B. Problem is that
                // in our next step, as we try to move from B->C, we are likely to be interrupted at B.
                // Until we apply this algorithm on a data structure that is robust to many vertices and
                // edges being right on top of each other, it's best to just terminate early.
                if( core::mathUtility::closeEnough( nextBarrierPos, hit->pos, 1e-3 ) ) {
                    nextSegID = boost::none;
                }
            } else {
                if( interruptingSegID == start.swd.metadata.segID ) {
                    // We've come back to the seg we started on.
                    ret.closed = true;
                } // else probably something went wrong, probably involving vertices on top of each other, but don't
                  // throw b/c I'm not sure I can guarantee no edge case bugs for now.
            }
            nextBarrierPos = hit->pos;
        } else {
            // Can we move to next segment on side of whatever 'Stroke'?                        
            const auto nextSegID_tentative = goWithBarr
                                                 ? curSWD.metadata.next
                                                 : curSWD.metadata.prev;
            const auto it = nextSegID_tentative
                ? _idToSWD.find( *nextSegID_tentative )
                : _idToSWD.end();
            if( it == _idToSWD.end() ) {
                // We've run all the way to the end of this side of 'Stroke'. This is the end.
            } else if( seenSegs.find( *nextSegID_tentative ) != seenSegs.end() ) {
                if( *nextSegID_tentative == start.swd.metadata.segID ) {
                    // Our path is a loop.
                    ret.closed = true;
                } // else probable something went wrong but don't throw ...
            } else {
                nextSegID = *nextSegID_tentative;
                seenSegs.emplace( *nextSegID );
                nextSWD = &it->second;
            }
        }

        auto normal = curSWD.metadata.normal;
        if( nextSWD ) {
            normal += nextSWD->metadata.normal;
            normal.normalize();
        }
        ret.pos.push_back( nextBarrierPos );
        ret.normal.push_back( normal );

        // Is there nowhere left to go along the on-barrier path
        if( nextSegID ) {
            curSegID = *nextSegID;
            if( !nextSWD ) THROW_UNEXPECTED;
            curSWD = *nextSWD;
        } else {
            break;
        }
    }

    if( ret.length() < 2 ) {
        return {};
    } else {
        return ret;
    }
}

} // mashup
