#include <tails/taildata.h>

#include <blendoptions.h>
#include <strokesegcollider.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/math/curveutility.h>
#include <Core/model/curveback.h>
#include <Core/model/lineback.h>

#include <Core/utility/mathutility.h>

namespace mashup {
namespace tails {

using Coll = StrokeSegCollider;
using Curve = core::model::Curve;
using OBP = OnBarrierPath;
using Opts = BlendOptions;
using Pos = core::model::Pos;
using Polyline = core::model::Polyline;
using Polylines = core::model::Polylines;
using Line = core::model::Line;
using Seg = core::model::Seg;
using UniqueCurve = core::model::UniqueCurve;

namespace {

UniqueCurve joinToFromPolyline( const Polyline& poly )
{
    switch( poly.size() ) {
    case 0: {
        THROW_UNEXPECTED;
    }
    case 1: {
        return Curve::lineSeg( poly.front(), poly.front() );
    }
    case 2: {
        return Curve::lineSeg( poly.front(), poly.back() );
    }
    case 3: {
        return Curve::spline( 2, poly );
    }
    case 4: {
        return Curve::spline( 3, poly );
    }
    default: {
        const size_t numSamples = std::max< size_t >(
            static_cast< size_t >( static_cast< double >( poly.size() ) * 1.5 ),
            10 );
        const auto resampledPoly = core::math::evenResamplePolyline( poly, numSamples );

        const int numControl = std::max< int >( 4, static_cast< int >( numSamples ) / 10 );
        return Curve::createFitToDataPoints( 3, numControl, resampledPoly );
    }
    }
    return nullptr;
}

} // unnamed

TailType TailData::tailType() const
{
    if( tTailAmount > 0. ) {
        return joinTo ? TailType::NormalTail : TailType::FallbackTail;
    } else {
        return TailType::NoTail;
    }
}

boost::optional< Pos > TailData::barrierHitPos() const
{
    if( obpBi.length() ) {
        return obpBi.pos[ obpBi_startIdx ];
    } else {
        return boost::none;
    }
}

void TailData::findJoinTo( const Stroke& s, const boost::optional< Line >& bisector, const Opts& opts, const Coll& coll )
{
    if( obpBi.length() == 0 ) {
        THROW_UNEXPECTED;
    }

    // 'trPoly'
    const size_t numSamples = 20;
    trPoly.resize( numSamples );
    trPolyT.resize( numSamples );
    for( size_t i = 0; i < numSamples; i++ ) {
        const auto strokeT = tr->tFromF( F_FROM_I( i, numSamples ) );
        const auto strokePos = s.curve().position( strokeT );
        trPoly[ i ] = strokePos;
        trPolyT[ i ] = strokeT;
    }

    const auto strokeLen = s.curve().cachedLength();

    // Find the radius to use for this end.
    tailRadius = 0.;
    {
        // What is width of 'stroke' around this end?
        double strokeWidth = 0.;
        if( tr.is_initialized() ) {
            const size_t numSamps = 5;
            for( size_t i = 0; i < numSamps; i++ ) {
                const auto strokeT = tr->tFromF( F_FROM_I( i, numSamps ) );
                strokeWidth += s.width( strokeT );
            }
            strokeWidth /= static_cast< double >( numSamps );
        } else {
            strokeWidth = s.width( endpoint == Start ? 0. : 1. );
        }

        tailRadius = std::max< double >(
            strokeWidth * 1.5,
            std::min< double >( strokeLen * 0.6, opts.tails.maxRad_canvas ) );
    }

    // Center of circle with rad 'radius'
    const auto circleCenter = *barrierHitPos();

    // The forward part of 'obpBi' that we want to preserve.
    bool hasLeftCircle = false; // as in for the first time (we don't worry about going in an out multiple times)
    double runAlongDist = 0.; // how far have we run after 'hasLeftCircle' has been set
    const auto maxRunAlongDist = opts.tails.maxOutsideCircle_f * tailRadius;
    const auto obp_forwardShort = obpBi.extractPolyline(
        obpBi_startIdx,
        true,
        [ & ]( const Pos& polyA, const Pos& polyB, Pos& storeEnd )
        {
            const auto& segA = polyA;
            auto segB = polyB;

            bool isLast = false;

            // Do we cross 'bisector'?
            if( bisector ) {
                const auto hit =
                    core::mathUtility::lineLineSegmentIntersection( bisector->a, bisector->b, segA, segB );
                if( hit ) {
                    segB = *hit;
                    isLast = true;
                }
            }

            if( hasLeftCircle ) {
                const auto dist = ( segB - segA ).length();
                if( runAlongDist + dist >= maxRunAlongDist ) {
                    isLast = true;
                    const auto amountAllowed = maxRunAlongDist - runAlongDist;
                    segB = segA + ( segB - segA ) * ( amountAllowed / dist );
                } else {
                    runAlongDist += dist;
                }
            } else {
                // Do we cross the circle on this step?
                const auto hits = core::mathUtility::lineSegmentCircleIntersection( segA, segB, circleCenter, tailRadius );
                if( hits.size() ) {
                    hasLeftCircle = true;
                    const auto& hit = hits.front();
                    runAlongDist = ( segB - hit ).length();
                    if( runAlongDist >= maxRunAlongDist ) {
                        const auto f = maxRunAlongDist / runAlongDist;
                        segB = hit + ( segB - hit ) * f;
                        isLast = true;
                    }
                }
            }

            if( isLast ) {
                storeEnd = segB;
                return true;
            } else {
                return false;
            }
        } );
    if( obp_forwardShort.size() < 1 ) {
        THROW_UNEXPECTED;
    } else if( obp_forwardShort.size() == 1 ) {
        return;
    }

    // The backward part of 'obpBi' that we want to preserve (we want it around so that
    // when we inflate, 's' won't be touching the rounded end cap, ideally).
    const auto obp_backShort = obpBi.extractPolyline(
        obpBi_startIdx,
        false,
        [ & ]( const Pos& polyA, const Pos& polyB, Pos& storeEnd )
        {
            bool isLast = false;
            // Do we cross 'bisector'?
            if( bisector ) {
                const auto hit =
                    core::mathUtility::lineLineSegmentIntersection( bisector->a, bisector->b, polyA, polyB );
                if( hit ) {
                    storeEnd = *hit;
                    isLast = true;
                }
            }

            if( !isLast ) {
                if( ( polyB - circleCenter ).length() >= tailRadius ) {
                    storeEnd = polyB;
                    isLast = true;
                }
            }

            return isLast;
        } );
    if( obp_backShort.size() < 1 ) {
        THROW_UNEXPECTED;
    }

    // Stitch "back" and "short" together to get something we can safely inflate.
    auto obp_toInflate = obp_backShort;
    std::reverse( obp_toInflate.begin(), obp_toInflate.end() );
    obp_toInflate.insert( obp_toInflate.end(), obp_forwardShort.begin() + 1, obp_forwardShort.end() );

    double offsetDist = opts.tails.maxOffsetDist_f * tailRadius;
    {
        // Measure how far 'tr' gets from 'obp_to_inflate' before 'tr' leaves the
        // circle of radius 'tailRadius' at the appropriate end of the blend-stroke.
        const auto& circleCenter = tr->endpoint( true );

        // What is the tailable region's maximum distance from the on-barrier path _before_
        // the tailable region leaves the circle of radius 'tailRadius' placed at its end?
        double maxDistFromToInflate = 0.;
        {
            bool exitedCircleYet = false; // only track the first exit from circle.
            const auto updateMaxDistFromInflate = [ & ]( const Pos& strokePos )
            {
                Pos unused;
                const auto distTo = core::mathUtility::distToPolyline( strokePos, obp_toInflate, unused );
                if( distTo > maxDistFromToInflate ) {
                    maxDistFromToInflate = distTo;
                }
            };

            for( size_t i = 0; i < trPoly.size(); i++ ) {
                const auto& strokePos = trPoly[ trPoly.size() - 1 - i ];
                if( ( strokePos - circleCenter ).length() > tailRadius ) {
                    // Is this the first time 'tr' leaves the circle?
                    if( !exitedCircleYet && i > 0 ) {
                        const auto& inCircle = trPoly[ trPoly.size() - i ];
                        const auto& outOfCircle = strokePos;
                        const auto hits = core::mathUtility::lineSegmentCircleIntersection( inCircle, outOfCircle, circleCenter, tailRadius );
                        if( hits.size() ) {
                            const auto& onCircle = hits.front();
                            updateMaxDistFromInflate( onCircle );
                        }
                    }
                    exitedCircleYet = true;
                    continue;
                }
                updateMaxDistFromInflate( strokePos );
            }
        }

        const double fac = 0.3; // keeping <= 0.5 should make this work properly in two-tailed small-space situations.
        offsetDist = std::min< double >( offsetDist, maxDistFromToInflate * fac );
    }

    if( offsetDist > 0. ) {
        const auto inflated = core::math::inflatePolyline( obp_toInflate, offsetDist );
        double storeCutoffT = 0.;
        Pos onWall = obp_forwardShort.back();
        Pos wallNorm = obp_forwardShort[ obp_forwardShort.size() - 2 ] - obp_forwardShort.back();
        wallNorm.normalize();
        joinTo = joinToFromInflated( inflated, storeCutoffT, onWall, wallNorm, offsetDist, coll );
        if( joinTo ) {
            tAtJoinTo = storeCutoffT;
        } else {
            // It may be tempting to set 'joinTo' to joinToFromPolyline( obp_forwardShort ), but
            // this isn't workable for now since connecting to said 'joinTo' will involve joints
            // that do not pass collision testing.
        }
    } else {
        THROW_RUNTIME( "offsetDist must be greater than 0" );
    }
}

UniqueCurve TailData::joinToFromInflated(
    const Polylines& polys,
    double& storeCutoffT,
    const Pos& onWall,
    const Pos& wallNormal,
    double offsetDist,
    const Coll& coll ) const
{
    if( trPoly.size() < 2 ) {
        THROW_UNEXPECTED;
    }

    // Walk from the end of 's' until first hitting one of 'polys'.
    size_t polyIdx = 0;
    size_t segIdx = 0;
    boost::optional< Pos > hitPos;
    double hitT = 0.; // on 's'
    bool segIdxIncreasing = false;
    /// Given pos-and-T values for an 's' segment moving AWAY from our endpoint, update 'hitPos'
    /// (and related vars) to reflect the first collision with 'polys' found along said seg (if
    /// there is at least one collision; otherwise leave aforementioned vars alone).
    const auto checkStrokeSeg = [ & ]( const Pos& pA, const Pos& pB, double tA, double tB )
    {
        double shortestDist = std::numeric_limits< double >::max(); // relative to 'pA' on the seg
        for( size_t pIdx = 0; pIdx < polys.size(); pIdx++ ) {
            const auto& poly = polys[ pIdx ];
            for( size_t vIdx = 0; vIdx < poly.size(); vIdx++ ) {
                const auto& polyA = poly[ vIdx ];
                const auto& polyB = poly[ ( vIdx + 1 ) % poly.size() ];
                Pos hit;
                if( core::mathUtility::segmentsIntersect( pA, pB, polyA, polyB, hit ) ) {
                    const auto distToHit = ( hit - pA ).length();
                    if( distToHit < shortestDist ) {
                        shortestDist = distToHit;
                        polyIdx = pIdx;
                        segIdx = vIdx;
                        hitPos = hit;
                        core::model::Seg sSeg{ pA, pB };
                        hitT = core::mathUtility::lerp( tA, tB, sSeg.t( hit ) );

                        // Note reversal of 'pA' and 'pB' here since we're walking _backwards_ along 's'.
                        segIdxIncreasing = !core::mathUtility::counterclockwise( pA - pB, polyB - polyA )
                                           == this->turnCW_atHit;
                    }
                }
            }
        }
    };

    const auto numTRSegs = trPoly.size() - 1;
    for( size_t i = 0; i < numTRSegs; i++ ) {
        // Walk backwards along 'tr' so that the first hit we find is
        // closest to our endpoint of 's'.
        const auto idxA = numTRSegs - i;
        const auto idxB = idxA - 1;
        const auto tA = trPolyT[ idxA ];
        const auto tB = trPolyT[ idxB ];
        const auto& pA = trPoly[ idxA ];
        const auto& pB = trPoly[ idxB ];
        checkStrokeSeg( pA, pB, tA, tB );
        if( hitPos ) {
            break;
        }
    }
    if( !hitPos ) {
        // It's possible that 's' actually ends too early to touch 'polys' at all (due to the
        // padding used to find the on-barrier path associated with this end of 's'). So we
        // still pick up a collision in this case, try adding a dummy, constant-T segment
        // connecting end of 'trPoly' (actual extent of 's') with the on-barrier point associated
        // with this end of 's'.
        const auto& trueEndOfS = trPoly.back();
        const auto& onBarrierPos = obpBi.pos[ obpBi_startIdx ];
        if( !core::mathUtility::closeEnough( trueEndOfS, onBarrierPos ) ) {
            checkStrokeSeg( onBarrierPos, trueEndOfS, trPolyT.back(), trPolyT.back() );
        }
        if( !hitPos ) {
            return nullptr;
        }
    }

    const auto& poly = polys[ polyIdx ];
    const auto numPolySegs = poly.size();
    Polyline curvePoints{ *hitPos };

    // Fill rest of 'curvePoints' from 'poly'
    auto wallDir = wallNormal;
    wallDir.turnPerpendicular();
    bool hasEnteredCircle = false;
    const double circleRad = offsetDist * 1.2;

    for( size_t i = 0; i < numPolySegs; i++ ) {
        const auto& last = curvePoints.back();
        auto next = segIdxIncreasing ? poly[ ( segIdx + 1 ) % poly.size() ] : poly[ segIdx ];
        bool nextIsLast = false;

        if( !hasEnteredCircle ) {
            if( ( next - onWall ).length() <= circleRad ) {
                hasEnteredCircle = true;
            } else {
                const auto hits = core::mathUtility::lineSegmentCircleIntersection( last, next, onWall, circleRad );
                if( hits.size() ) {
                    hasEnteredCircle = true;
                }
            }
        }

        if( hasEnteredCircle ) {
            // Does 'last' -> 'next' take us past 'wall'?
            const auto hitWall = core::mathUtility::lineLineSegmentIntersection( onWall, onWall + wallDir, last, next );
            if( hitWall ) {
                if( Pos::dot( ( next - last ), wallNormal ) < 0. ) {
                    // We've collided with the wall in the expected direction.
                    nextIsLast = true;
                    next = *hitWall;
                }
            }
        }

        curvePoints.push_back( next );
        if( nextIsLast ) {
            break;
        }

        if( segIdxIncreasing ) {
            segIdx = ( segIdx + 1 ) % numPolySegs;
        } else {
            if( segIdx == 0 ) {
                segIdx = numPolySegs - 1;
            } else {
                segIdx--;
            }
        }
    }

    if( curvePoints.size() > 1 ) {
        auto curve = joinToFromPolyline( curvePoints );
        curve = limitByCollider( *curve, coll );
        storeCutoffT = hitT;
        return curve;
    } else {
        return nullptr;
    }
}

UniqueCurve TailData::limitByCollider( const Curve& toLimit, const Coll& coll ) const
{
    const size_t numSamples = 20;
    std::vector< double > sampleT( numSamples );
    Polyline sampleP( numSamples );
    for( size_t i = 0; i < numSamples; i++ ) {
        sampleT[ i ] = F_FROM_I( i, numSamples );
        sampleP[ i ] = toLimit.position( sampleT[ i ] );
    }

    boost::optional< double > endT;
    for( size_t i = 0; i < numSamples - 1; i++ ) {
        const auto& pA = sampleP[ i ];
        const auto& pB = sampleP[ i + 1 ];
        const Seg pSeg{ pA, pB };
        const auto hit = coll.firstHit( pSeg, nullptr, true );
        if( hit ) {
            const auto tA = sampleT[ i ];
            const auto tB = sampleT[ i + 1 ];
            endT = core::mathUtility::lerp( tA, tB, hit->fHitter );
            break;
        }
    }

    if( endT ) {
        return toLimit.extractCurveForTInterval( 0., *endT );
    } else {
        return toLimit.clone();
    }
}

void TailData::findHitAndOBP( const Stroke& s, const Coll& coll, const Opts& opts )
{
    Pos rayDir;
    if( endpoint == Start ) {
        rayDir = s.curve().derivative( 0. ) * -1.;
    } else {
        rayDir = s.curve().derivative( 1. );
    }
    rayDir.normalize();

    // So far haven't seen a definite need for this to be >0, plus it's easy to imagine
    // cases where this could cause bad behavior.
    const auto safetyShiftBack = 0.;

    const auto& epPos = endpoint == Start ? s.curve().startPosition() : s.curve().endPosition();

    auto rayStart = epPos;
    // Put the start position of the ray a little behind the actual endpoint in case the end of 's'
    // is right on top of or even slightly past the barrier we want to find a collision with.
    rayStart -= rayDir * safetyShiftBack;

    // ltwarning: This shows a flaw in our current tail-generation approach.
    //      Consider a situation where a wide stroke strikes a (wider) barrier at an oblique angle.
    //      The ray might hit that barrier a very long way off; in fact, if the stroke is wide enough,
    //      the ray might not hit that barrier at all, but might strike some barrier very far off
    //      that definitely should not be involved with this tail. So, if we use a too-short 'safeShift-
    //      Forward', then we risk having no tail (rather, a "fallback" tail) in a case where we obvious-
    //      ly should have been able to construct a tail running along aforementioned wall. But if
    //      we set the safety shift too high, we risk constructing a tail in clearly the wrong place,
    //      when we instead should obviously have created a fallback tail.
    //
    double safetyShiftForward = 0.;
    {
        // at endpoint
        const auto sWidth = endpoint == Start ? s.width( 0. ) : s.width( 1. );
        safetyShiftForward = std::max< double >(
            {
             safetyShiftBack,
             sWidth * 1.5,
             s.curve().cachedLength(),
             opts.tails.maxRad_canvas } );
    }
    const auto rayEnd = epPos + rayDir * safetyShiftForward;

    obpBi = coll.onBarrierPath( Seg{ rayStart, rayEnd }, true, obpBi_startIdx );
    if( obpBi.length() ) {
        // Does 's' turn clockwise into its tail?
        turnCW_atHit = !core::mathUtility::counterclockwise( rayDir, obpBi.dir( obpBi_startIdx ) );
    }
}

} // tails
} // mashup
