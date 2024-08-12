#include <tails/tailmaker.h>

#include <blenddrawings.h>
#include <blendoptions.h>
#include <endpoint.h>
#include <onbarrierpath.h>
#include <paircutter.h>
#include <strokesegcollider.h>
#include <substroke.h>
#include <tails/taildata.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/math/curveutility.h>
#include <Core/math/interpcubic.h>
#include <Core/model/curveback.h>
#include <Core/model/stroketools.h>

#include <Core/utility/bspline2utility.h>

#include <array>

namespace mashup {
namespace tails {

namespace {

constexpr size_t CutTries = 12;

/// When joining a 'Curve'/'Stroke' 'x' in a join that shouldn't consume all
/// of 'x', preserve this much of it (in T-space).
constexpr double preserveStroke = 0.1;

/// When 'BoundingBox' 'a' is supposed to "survive", at least this much of its
/// area (out of 1.) must be preserved.
constexpr double boundingBoxMinSurvivalRatio = 0.8;

} // unnamed

using BoundingBox = core::BoundingBoxd;
using BoundingInterval = core::BoundingIntervald;
using Coll = StrokeSegCollider;
using CutRange = BoundingInterval;
using Cutter = PairCutter< CutTries >;
using Curve = core::model::Curve;
using OBP = OnBarrierPath;
using Opts = BlendOptions;
using Pos = core::model::Pos;
using Polyline = core::model::Polyline;
using Polylines = core::model::Polylines;
using PreserveInterval = TailMaker::PreserveInterval;
using Line = core::model::Line;
using RawConstCurves = core::model::RawConstCurves;
using Seg = core::model::Seg;
using UniqueCurve = core::model::UniqueCurve;
using UniqueStroke = core::model::UniqueStroke;

namespace {

/// Return the unioned of the bounding boxes of all of 'curves' (size >0).
BoundingBox boundingBoxUnion( const core::model::RawConstCurves& curves )
{
    BoundingBox ret = curves.front()->boundingBox();
    for( size_t i = 1; i < curves.size(); i++ ) {
        ret.growToContain( curves[ i ]->boundingBox() );
    }
    return ret;
}

/// Return whether enough of 'a' survives in its intersection with 'b'.
bool aMostlySurvivesInB( const BoundingInterval& a, const BoundingInterval& b )
{
    const auto aArea = a.length();
    if( core::mathUtility::closeEnoughToZero( aArea ) ) {
        return true;
    }
    const auto intersect = BoundingInterval::intersection( a, b );
    if( intersect ) {
        return ( intersect->length() / aArea ) >= boundingBoxMinSurvivalRatio;
    } else {
        return false;
    }
}

/// Return whether enough of 'a' survives in its intersection with 'b'.
bool aMostlySurvivesInB( const BoundingBox& a, const BoundingBox& b )
{
    const auto aArea = a.size();
    if( core::mathUtility::closeEnoughToZero( aArea ) ) {
        // Fall back to treating them as 1D intervals.
        if( a.widthExclusive() > a.heightExclusive() ) {
            return aMostlySurvivesInB( a.xInterval(), b.xInterval() );
        } else {
            return aMostlySurvivesInB( a.yInterval(), b.yInterval() );
        }
    }
    const auto intersect = BoundingBox::intersection( a, b );
    if( intersect ) {
        return ( intersect->size() / aArea ) >= boundingBoxMinSurvivalRatio;
    } else {
        return false;
    }
}

} // unnamed

/// The parts used to construct a tail-ified version of 'Stroke' 's'.
struct ResultParts
{
    void throwIfInvalid( const Stroke& s ) const
    {
        if( start_joint ) {
            if( !start_fromJoinTo ) {
                THROW_UNEXPECTED;
            }
            if( start_fallbackTail ) {
                THROW_UNEXPECTED;
            }
        }

        if( mid_joint ) {
            if( mid_t || mid_tPreserve ) {
                THROW_UNEXPECTED;
            }
            if( start_joint || end_joint ) {
                THROW_UNEXPECTED;
            }
            if( !start_fromJoinTo || !end_fromJoinTo ) {
                THROW_UNEXPECTED;
            }
        } else {
            if( !mid_t ) {
                THROW_UNEXPECTED;
            }
            if( mid_t->length() == 0. ) {
                THROW_UNEXPECTED;
            }
            if( mid_tPreserve ) {
                if( mid_tPreserve->min() < mid_t->min() ) {
                    THROW_UNEXPECTED;
                }
                if( mid_tPreserve->max() > mid_t->max() ) {
                    THROW_UNEXPECTED;
                }
            }
        }

        if( end_joint ) {
            if( end_fallbackTail ) {
                THROW_UNEXPECTED;
            }
            if( !end_fromJoinTo ) {
                THROW_UNEXPECTED;
            }
        }

        if( hasBadStitch( s, 1. ) ) {
            THROW_UNEXPECTED;
        }
    }

    /// Return whether the pieces specified in 'this' are in fact a C0 sequence.
    bool hasBadStitch( const Stroke& s, double thresh ) const
    {
        std::vector< const Curve* > curvesToStitch;
        UniqueCurve fromStroke;

        if( start_fromJoinTo ) {
            curvesToStitch.push_back( start_fromJoinTo.get() );
        }
        if( start_joint ) {
            curvesToStitch.push_back( start_joint.get() );
        }

        // MIDDLE
        if( mid_joint ) {
            curvesToStitch.push_back( mid_joint.get() );
        } else {
            fromStroke = s.curve().extractCurveForTInterval( mid_t->min(), mid_t->max() );
            curvesToStitch.push_back( fromStroke.get() );
        }

        if( end_joint ) {
            curvesToStitch.push_back( end_joint.get() );
        }
        if( end_fromJoinTo ) {
            curvesToStitch.push_back( end_fromJoinTo.get() );
        }

        return !core::math::curvesAreApproxC0( curvesToStitch, false, thresh );
    }

    ///////////////////////////////////////
    /// START
    ////////////////////////////////////////

    /// Only true if 'start_fromJoinTo' and 'start_joint' are nullptr.
    bool start_fallbackTail = false;
    /// Ends at start of 'start_joint'.
    UniqueCurve start_fromJoinTo;
    /// Heads towards middle part (or end part if no middle part).
    /// Set only if 'fromJoinToStart' is non-null and if 'tFromStroke' is set.
    UniqueCurve start_joint;

    ///////////////////////////////////////
    /// MIDDLE
    ////////////////////////////////////////

    /// Must be set if 'mid_joint' is nullptr. This is a T-interval on 's'.
    PreserveInterval mid_t;
    /// Can be set only if 'mid_t' is set, and only to a subinterval of 'mid_t' (it may equal 'mid_t').
    PreserveInterval mid_tPreserve;
    /// Set only if 'start_joint' and 'end_joint' are nullptr and if 'start_fromJoinTo' and 'end_fromJoinTo'
    /// are not nullptr and if 'mid_tFromStroke' not set.
    UniqueCurve mid_joint;

    ///////////////////////////////////////
    /// END
    ////////////////////////////////////////

    /// Starts at end of middle part (or end of start part).
    /// Set only if 'end_fromJoinTo' is non-null and if 'mid_tFromStroke' is set.
    UniqueCurve end_joint;
    /// Starts at end of 'end_joint'.
    UniqueCurve end_fromJoinTo;
    /// Only true if 'end_joint' and 'end_fromJoinTo' are nullptr.
    bool end_fallbackTail = false;
};

struct TailMaker::Imp
{
    Imp( const Stroke& s,
        const PreserveInterval& preserveMid,
        const Coll& c,
        const BlendDrawings& bd )
        : stroke( s )
        , preserveMid( preserveMid )
        , coll( c )
        , blendDrawings( bd )
        , opts( bd.options() )
    {
    }

    /// Return true if 'c' doesn't hit anything in 'coll'.
    bool curveCollisionFree( const Curve& c ) const
    {
        // Check 'joint' to see whether it hits anything.
        const size_t numSamples = 20;
        const auto cPoly = c.crudePolylineApproximation( numSamples );
        return !coll.hitsAnything( cPoly );
    }

    /// Return the result for when 'stroke' is not _supposed_ to have any tails.
    /// In practice this should never be called; 'this''s parent is unneeded
    /// in such cases.
    ResultParts resultParts_noTails()
    {
        ResultParts ret;
        ret.mid_t = BoundingInterval{ 0., 1. };
        ret.mid_tPreserve = preserveMid;
        return ret;
    }

    /// Build results for a case where only one of the ends (identified by
    /// 'start') can have a 'NormalTail' tail.
    ResultParts resultParts_oneNormalTail( bool start )
    {
        ResultParts ret;
        ret.mid_tPreserve = preserveMid;

        if( start ) {
            // START TAIL

            if( preserveMid && tailData[ Start ].tAtJoinTo >= preserveMid->min() ) {
                THROW_UNEXPECTED;
            }

            const auto joinToStart = tailData[ Start ].joinTo->reverseCopy();

            // Where can we place a cut-T in 'joinToStart'?
            const CutRange cr_joinToStart
            {
                std::max< double >( preserveStroke,
                                 core::math::eraseCircleT( *joinToStart, tailData[ Start ].tailRadius, false ) ),
                1.
            };

            CutRange cr_stroke;
            {
                const auto min = tailData[ Start ].tAtJoinTo;
                const BoundingInterval fromStrokeInterval{ tailData[ Start ].tAtJoinTo, 1. };
                auto fromStroke = stroke.curve().extractCurveForTInterval( fromStrokeInterval.min(), fromStrokeInterval.max() );
                auto max = core::math::eraseCircleT( *fromStroke, tailData[ Start ].tailRadius, true );
                max = std::min< double >( 1. - preserveStroke, fromStrokeInterval.lerp( max ) );
                if( preserveMid ) {
                    max = std::min< double >( max, preserveMid->min() );
                }
                cr_stroke = { min, max };
            }

            const auto boundsJoinTo = joinToStart->boundingBox();

            // Try various strengths of cut, strongest to weakest.
            bool success = false;
            for( size_t i = 0; i < CutTries; i++ ) {
                // largest 'f' first
                const auto f = 1. - F_FROM_I( i, CutTries );

                auto fromJoinToStart = joinToStart->extractCurveForTInterval(
                    0.,
                    cr_joinToStart.lerp( 1. - f ) );

                const auto t_stroke = std::min< double >(
                    preserveMid ? preserveMid->min() : 1.,
                    cr_stroke.lerp( f ) );

                const auto fromStroke = stroke.curve().extractCurveForTInterval( t_stroke, 1. );

                auto joint = core::math::smoothJoint( *fromJoinToStart, *fromStroke );
                if( !joint ) {
                    continue;
                }

                const auto shouldBoundJoinTo = boundingBoxUnion(
                    core::model::RawConstCurves
                    {
                        fromJoinToStart.get(),
                        joint.get()
                    } );
                if( !aMostlySurvivesInB( boundsJoinTo, shouldBoundJoinTo ) ) {
                    continue;
                }
                if( !curveCollisionFree( *joint ) ) {
                    continue;
                }

                success = true;
                ret.start_fromJoinTo = std::move( fromJoinToStart );
                ret.start_joint = std::move( joint );
                ret.mid_t = BoundingInterval{ t_stroke, 1. };
                break;
            }
            if( !success ) {
                auto ret = resultParts_noTails();
                ret.start_fallbackTail = true;
                return ret;
            } else {
                return ret;
            }
        } else {
            // END TAIL

            if( preserveMid && tailData[ End ].tAtJoinTo <= preserveMid->max() ) {
                THROW_UNEXPECTED;
            }

            const auto& joinToEnd = tailData[ End ].joinTo;

            // In what T-range can we cut 'joinToEnd'?
            const CutRange cr_joinToEnd
                {
                    0.,
                    std::min< double >( 1. - preserveStroke,
                                 core::math::eraseCircleT( *joinToEnd, tailData[ End ].tailRadius, true ) )
                };

            CutRange cr_stroke;
            {
                const BoundingInterval fromStrokeInterval{ 0., tailData[ End ].tAtJoinTo };
                auto fromStroke = stroke.curve().extractCurveForTInterval( fromStrokeInterval.min(), fromStrokeInterval.max() );

                const auto maxT = tailData[ End ].tAtJoinTo;
                auto minT = core::math::eraseCircleT( *fromStroke, tailData[ End ].tailRadius, false );
                minT = std::max< double >( preserveStroke, fromStrokeInterval.lerp( minT ) );
                if( preserveMid ) {
                    minT = std::max< double >( preserveMid->max(), minT );
                }
                cr_stroke = { minT, maxT };
            }

            const auto boundsJoinTo = joinToEnd->boundingBox();

            bool success = false;
            for( size_t i = 0; i < CutTries; i++ ) {
                const auto f = 1. - F_FROM_I( i, CutTries );

                auto fromJoinToEnd = joinToEnd->extractCurveForTInterval(
                    cr_joinToEnd.lerp( f ),
                    1. );

                const auto t_stroke = std::max< double >(
                    preserveMid ? preserveMid->max() : 0.,
                    cr_stroke.lerp( 1. - f ) );

                const auto fromStroke = stroke.curve().extractCurveForTInterval( 0., t_stroke );

                auto joint = core::math::smoothJoint( *fromStroke, *fromJoinToEnd );
                if( !joint ) {
                    continue;
                }

                // Bounding box produced by the join
                const auto shouldBoundJoinTo = boundingBoxUnion(
                    core::model::RawConstCurves
                    {
                        joint.get(),
                        fromJoinToEnd.get()
                    } );
                if( !aMostlySurvivesInB( boundsJoinTo, shouldBoundJoinTo ) ) {
                    continue;
                }
                if( !curveCollisionFree( *joint ) ) {
                    continue;
                }

                success = true;
                ret.mid_t = BoundingInterval{ 0., t_stroke };
                ret.end_joint = std::move( joint );
                ret.end_fromJoinTo = std::move( fromJoinToEnd );
                break;
            }
            if( success ) {
                return ret;
            } else {
                auto ret = resultParts_noTails();
                ret.end_fallbackTail = true;
                return ret;
            }
        }
    }

    /// Build results when both ends want and can have 'NormalTail' tails.
    ResultParts resultParts_twoNormalTails()
    {
        if( tailData[ Start ].tAtJoinTo >= tailData[ End ].tAtJoinTo ) {
            // ltwarning: It's probably possible to engineer this out of possibility, but
            // for now it feels like an edgy enough case to just handle gracefully at runtime.
            auto ret = resultParts_noTails();
            ret.start_fallbackTail = true;
            ret.end_fallbackTail = true;
            return ret;
        }
        if( preserveMid ) {
            if( tailData[ Start ].tAtJoinTo >= preserveMid->min() ) {
                THROW_UNEXPECTED;
            }
            if( tailData[ End ].tAtJoinTo <= preserveMid->max() ) {
                THROW_UNEXPECTED;
            }
        }

        // A T-interval on 'stroke' between the points where it connects to the two join-to curves.
        const BoundingInterval strokeBetween{ tailData[ Start ].tAtJoinTo, tailData[ End ].tAtJoinTo };
        
        const auto startRad = tailData[ Start ].tailRadius;
        const auto endRad = tailData[ End ].tailRadius;
        if( startRad <= 0. || endRad <= 0. ) {
            THROW_UNEXPECTED;
        }

        // 'mid' is a piece of 'Stroke' that we're going to connect to something at both ends.
        // It may end up being completely ignored in the double-join.
        const auto midStroke = stroke.strokeInterval( tailData[ Start ].tAtJoinTo, tailData[ End ].tAtJoinTo );
        const auto& midCurve = midStroke->curve();
        const auto joinToStart = tailData[ Start ].joinTo->reverseCopy(); // ends at start of 'midStroke'
        const auto& joinToEnd = tailData[ End ].joinTo;

        // In what T-range can we cut 'joinToStart'?
        const CutRange cr_joinToStart
        {
            std::max< double >( preserveStroke,
                             core::math::eraseCircleT( *joinToStart, tailData[ Start ].tailRadius, false  ) ),
            1.
        };
        // In what T-range can we place the first cut on 'stroke' itself?
        CutRange cr_strokeStart;
        {
            const auto minT = tailData[ Start ].tAtJoinTo;
            auto maxT = core::math::eraseCircleT( midCurve, tailData[ Start ].tailRadius, true );
            maxT = strokeBetween.lerp( maxT );
            if( preserveMid ) {
                maxT = std::min< double >( preserveMid->min(), maxT );
            }
            cr_strokeStart = { minT, maxT };
        }
        // In what T-range can we place the second cut on 'stroke' itself?
        CutRange cr_strokeEnd;
        {
            const auto maxT = tailData[ End ].tAtJoinTo;
            auto minT = core::math::eraseCircleT( midCurve, tailData[ End ].tailRadius, false );
            minT = strokeBetween.lerp( minT );
            if( preserveMid ) {
                minT = std::max< double >( preserveMid->max(), minT );
            }
            cr_strokeEnd = { minT, maxT };
        }
        // In what T-range can we cut 'joinToEnd'?
        const CutRange cr_joinToEnd
        {
            0.,
            std::min< double >( 1. - preserveStroke,
                             core::math::eraseCircleT( *joinToEnd, tailData[ End ].tailRadius, true ) )
        };

        const auto boundsJoinToStart = joinToStart->boundingBox();
        const auto boundsJoinToEnd = joinToEnd->boundingBox();

        ResultParts ret;
        ret.mid_tPreserve = preserveMid;
        const auto findCutsResult = Cutter::doUntilSuccess(
        /// 'fStart' and 'fEnd' in [0,1]. Larger values mean larger cuts.
        [ & ]( double fStart, double fEnd )
        {
            auto fromJoinToStart = joinToStart->extractCurveForTInterval(
                    0.,
                    cr_joinToStart.lerp( 1. - fStart ) );
            const auto fromJoinToStart_bounds = fromJoinToStart->boundingBox();
            auto fromJoinToEnd = joinToEnd->extractCurveForTInterval(
                    cr_joinToEnd.lerp( fEnd ),
                    1. );
            const auto fromJoinToEnd_bounds = fromJoinToEnd->boundingBox();

            // Get our two cut-T values on 'stroke' and make sure fp error
            // doesn't carry these into 'preserveMid'.
            const auto t_strokeStart = std::min< double >(
                preserveMid ? preserveMid->min() : 1.,
                cr_strokeStart.lerp( fStart ) );
            const auto t_strokeEnd = std::max< double >(
                preserveMid ? preserveMid->max() : 0.,
                cr_strokeEnd.lerp( 1. - fEnd ) );

            /// Return whether
            ///     'boundsJoinToStart' survives enough in box containing 'startJoint' and 'fromJoinToStart'
            /// and whether
            ///     'boundsJoinToEnd' survives enough in box containing 'endJoint' and 'fromJoinToEnd'
            const auto boundsGood = [ & ]( const Curve& startJoint, const Curve& endJoint )
            {
                BoundingBox box;
                // (1)
                box = fromJoinToStart_bounds;
                box.growToContain( startJoint.boundingBox() );
                if( !aMostlySurvivesInB( boundsJoinToStart, box ) ) {
                    return false;
                }
                // (2)
                box = fromJoinToEnd_bounds;
                box.growToContain( endJoint.boundingBox() );
                if( !aMostlySurvivesInB( boundsJoinToEnd, box ) ) {
                    return false;
                }
                return true;
            };

            if( t_strokeStart < t_strokeEnd ) {
                // We're keeping some of 'stroke', which means there are two joints to test.
                const auto fromStroke = stroke.curve().extractCurveForTInterval( t_strokeStart, t_strokeEnd );
                auto jointStart = core::math::smoothJoint( *fromJoinToStart, *fromStroke );
                if( !jointStart ) {
                    return false;
                }
                auto jointEnd = core::math::smoothJoint( *fromStroke, *fromJoinToEnd );
                if( !jointEnd ) {
                    return false;
                }

                if( !boundsGood( *jointStart, *jointEnd ) ) {
                    return false;
                }
                if( !curveCollisionFree( *jointStart ) ) {
                    return false;
                }
                if( !curveCollisionFree( *jointEnd ) ) {
                    return false;
                }

                ret.start_fromJoinTo = std::move( fromJoinToStart );
                ret.start_joint = std::move( jointStart );
                ret.mid_t = BoundingInterval{ t_strokeStart, t_strokeEnd };
                ret.end_joint = std::move( jointEnd );
                ret.end_fromJoinTo = std::move( fromJoinToEnd );
                return true;
            } else {
                if( preserveMid ) {
                    THROW_UNEXPECTED;
                }

                // We're not going to make two separate joints with a piece of 'stroke'
                // in between; instead we're going to make a single joint connecting
                // 'fromJoinToStart' to 'fromJoinToEnd'. This joint will, however,
                // go through a _point_ on 'stroke' (this keeps the joint from being pulled
                // way to the side).
                const auto pointOnStroke =
                    stroke.curve().position( ( t_strokeStart + t_strokeEnd ) / 2. );

                auto joint = core::math::smoothJoint( *fromJoinToStart, pointOnStroke, *fromJoinToEnd );
                if( !joint ) {
                    return false;
                }

                if( !boundsGood( *joint, *joint ) ) {
                    return false;
                }
                if( !curveCollisionFree( *joint ) ) {
                    return false;
                }

                ret.start_fromJoinTo = std::move( fromJoinToStart );
                ret.mid_joint = std::move( joint );
                ret.end_fromJoinTo = std::move( fromJoinToEnd );
                return true;
            }
        } );

        if( findCutsResult ) {
            return ret;
        } else {
            auto ret = resultParts_noTails();
            ret.start_fallbackTail = true;
            ret.end_fallbackTail = true;
            return ret;
        }
    }

    UniqueStroke result()
    {
        // Get initial info about endpoints
        for( int e = 0; e < NumEndpoints; e++ ) {
            auto& endpoint = tailData[ e ];
            endpoint.endpoint = static_cast< Endpoint >( e );
            if( e == Start ) {
                endpoint.tAtJoinTo = 0.;
                endpoint.tTailAmount = preserveMid ? std::max< double >( 0., preserveMid->min() ) : 1.;
                if( endpoint.tTailAmount > 0. ) {
                    endpoint.tr = Substroke( stroke, endpoint.tTailAmount, 0. );
                }
            } else {
                endpoint.tAtJoinTo = 1.;
                endpoint.tTailAmount = preserveMid ? 1. - std::min< double >( 1., preserveMid->max() ) : 1.;
                if( endpoint.tTailAmount > 0. ) {
                    endpoint.tr = Substroke( stroke, 1. - endpoint.tTailAmount, 1. );
                }
            }

            if( endpoint.tTailAmount > 0. ) {
                endpoint.findHitAndOBP( stroke, coll, opts );
            }
        }

        // Find 'bisector' if needed
        if( tailData[ Start ].obpBi.length()
         && tailData[ End ].obpBi.length() ) {
            
            const auto& hitA = *tailData[ Start ].barrierHitPos();
            const auto& hitB = *tailData[ End ].barrierHitPos();
            const auto center = ( hitA + hitB ) / 2.;
            auto lineDir = hitB - hitA;
            lineDir.turnPerpendicular();
            bisector = Line{ center, center + lineDir };
        }

        // Find the join-to paths for those endpoints that need them
        for( int e = 0; e < NumEndpoints; e++ ) {
            if( tailData[ e ].obpBi.length() ) {
                tailData[ e ].findJoinTo( stroke, bisector, opts, coll );
            }
        }

        const auto startType = tailData[ Start ].tailType();
        const auto endType = tailData[ End ].tailType();

        ResultParts resultParts;
        if( startType == NoTail && endType == NoTail ) {
            resultParts = resultParts_noTails();
        } else {
            if( startType == NormalTail && endType == NormalTail ) {
                resultParts = resultParts_twoNormalTails();
            } else if( startType == NormalTail || endType == NormalTail ) {
                resultParts = resultParts_oneNormalTail( startType == NormalTail );
            } else {
                resultParts = resultParts_noTails();
            }
        }
        if( startType == FallbackTail ) {
            resultParts.start_fallbackTail = true;
        }
        if( endType == FallbackTail ) {
            resultParts.end_fallbackTail = true;
        }
        // Check for errors.
        resultParts.throwIfInvalid( stroke );
        return resultFromParts( resultParts );
    }

    /// Return a 'Stroke' whose path is made from the C0+ chain 'curveParts' and whose width curve is based on
    ///     (1) original 'stroke' widths from the interval 'tStrokeStart' to 'tStrokeEnd'
    ///     (2) local tapering based on path curve's proximity to barriers in 'coll'
    ///     (3) 'overallTaper' (all in [0,1], size is length of 'curveParts' + 1;
    ///                         each interval in 'overallTaper' represents one of 'curveParts')
    /// If set, 'constrainEnd' requires the returned stroke to have the indicated width at the indicated end.
    UniqueStroke taperedStroke(
        const std::vector< const Curve* >& curveParts,
        const std::vector< double >& overallTaper,
        double tStrokeStart,
        double tStrokeEnd,
        const boost::optional< std::pair< Endpoint, double > >& constrainEnd ) const
    {
        if( curveParts.size() < 1 ) {
            THROW_UNEXPECTED;
        }

        // This maps from F in [0,1] (the full interval of the 'Stroke' we are building)
        // to interpolated values from 'overallTaper'
        std::unique_ptr< core::math::InterpCubic > overallTaperFromF;
        {
            /// X is distance along 'posCurve' while Y is [0,1] from 'overallTaper'.
            using XY = core::math::InterpCubic::XY;
            using XYs = core::math::InterpCubic::XYs;

            double distAlongPosCurve = 0.;
            XYs xy;
            xy.push_back( XY{ distAlongPosCurve, overallTaper.front() } );

            for( size_t i = 0; i < curveParts.size(); i++ ) {
                distAlongPosCurve += curveParts[ i ]->cachedLength();
                xy.push_back( XY{ distAlongPosCurve, overallTaper[ i + 1 ] } );
            }
            overallTaperFromF = std::make_unique< core::math::InterpCubic >( xy );
        }

        auto posCurve = core::BSpline2Utility::stitchC0Spline( curveParts, Curve::defaultLengthPrecision );

        size_t numWSamples = 0;
        {
            const auto len = posCurve->cachedLength();
            numWSamples = std::max< size_t >(
                static_cast< size_t >( ( len / coll.bounds().avgDim() ) * 40. ),
                curveParts.size() * 3 );
        }
        Polyline wControl( numWSamples );
        for( size_t i = 0; i < numWSamples; i++ ) {
            const auto tPosCurve = F_FROM_I( i, numWSamples );
            const auto tStroke = core::mathUtility::lerp( tStrokeStart, tStrokeEnd, tPosCurve );

            const auto width_stroke = stroke.width( tStroke );
            double width_maxAllowedByColl = width_stroke;
            {
                // The *2 and /2 reflect switching between stroke width and stroke "radius".
                const auto pos = posCurve->position( tPosCurve );
                const auto res = coll.distToNearestSeg(
                    pos, nullptr, ( width_stroke * 0.5 ) / opts.tails.maxWidthFillAllowed_f );
                if( res ) {
                    width_maxAllowedByColl = 2. * ( res.value() * opts.tails.maxWidthFillAllowed_f );
                }
            }

            const auto fTaper = overallTaperFromF->yFromF( tPosCurve );
            const auto width = std::min< double >( width_stroke, width_maxAllowedByColl ) * fTaper;
            wControl[ i ] = Pos{ tPosCurve, width };
        }

        if( constrainEnd ) {
            // Force width to be something at one end or the other.
            const auto whichEnd = constrainEnd->first;
            const auto forceWidth = constrainEnd->second;

            for( size_t i = 0; i < numWSamples; i++ ) {
                const auto f = 1. - F_FROM_I( i, numWSamples );

                const auto idx = whichEnd == Start ? i : numWSamples - 1 - i;

                const auto oldW = wControl[ idx ].y();
                const auto newW = core::mathUtility::lerp( oldW, forceWidth, std::pow( f, 1.5 ) );
                wControl[ idx ].setY( newW );
            }
        }

        // Keep the width curve consistent with the position curve (assumption here is that
        // our width samples have been evenly distributed vis-a-vis 'posCurve' arclength).
        std::vector< double > wInternalT( wControl.size() - 2 );
        for( size_t i = 0; i < wInternalT.size(); i++ ) {
            wInternalT[ i ] = F_FROM_I( i + 1, wControl.size() );
        }

        auto wCurve = Curve::naturalInterpolation( wControl, wInternalT );
        return core::model::strokeFromPosAndWidth( std::move( posCurve ), std::move( wCurve ) );
    }

    /// 'rp' must be valid
    UniqueStroke resultFromParts( const ResultParts& rp ) const
    {
        if( rp.mid_joint ) {
            // Special case where we have no sub-'Stroke' taken directly from 'stroke'.
            const RawConstCurves curveParts
            {
                rp.start_fromJoinTo.get(),
                rp.mid_joint.get(),
                rp.end_fromJoinTo.get()
            };
            return taperedStroke(
                curveParts,
                { 0., 1., 1., 0. },
                0.,
                1.,
                boost::none );
        }

        auto fromStroke = stroke.strokeInterval( rp.mid_t->min(), rp.mid_t->max() );

        // START PART (before 'fromStroke')
        UniqueStroke toFromStroke;
        if( rp.start_fromJoinTo ) {
            const RawConstCurves curveParts
            {
                rp.start_fromJoinTo.get(),
                rp.start_joint.get()
            };
            toFromStroke = taperedStroke(
                curveParts,
                { 0., 1., 1. },
                0.,
                rp.mid_t->min(),
                std::pair< Endpoint, double >{ End, fromStroke->width( 0. ) } );
        }

        // END PART (after 'fromStroke')
        UniqueStroke fromFromStroke;
        if( rp.end_fromJoinTo ) {
            const RawConstCurves curveParts
            {
                rp.end_joint.get(),
                rp.end_fromJoinTo.get()
            };
            fromFromStroke = taperedStroke(
                curveParts,
                { 1., 1., 0. },
                rp.mid_t->max(),
                1.,
                std::pair< Endpoint, double >{ Start, fromStroke->width( 1. ) } );
        }

        core::model::UniqueStrokes toStitch;
        if( toFromStroke ) {
            toStitch.push_back( std::move( toFromStroke ) );
        }
        toStitch.push_back( std::move( fromStroke ) );
        if( fromFromStroke ) {
            toStitch.push_back( std::move( fromFromStroke ) );
        }
        auto stitched = core::model::stitchC0Strokes( toStitch, false );

        // Fallback tails
        if( rp.start_fallbackTail || rp.end_fallbackTail ) {
            const auto taperRad = std::min< double >(
                stitched->curve().cachedLength() * 0.3,
                opts.tails.maxRad_canvas );

            boost::optional< double > tA, tB;
            if( rp.start_fallbackTail ) {
                tA = core::math::eraseCircleT( stitched->curve(), taperRad, true );
            }
            if( rp.end_fallbackTail ) {
                tB = core::math::eraseCircleT( stitched->curve(), taperRad, false );
            }
            stitched = core::model::taperStrokeEndpoints( *stitched, tA, tB );
        }

        return stitched;
    }

    const Stroke& stroke;
    /// An interval on 'stroke' that must be preserved in the result.
    const PreserveInterval preserveMid;
    const Coll& coll;
    const BlendDrawings& blendDrawings;
    const Opts& opts;

    /// Used in two-normal-cases cases to spatially separate tails and
    /// prevent weird overlaps in enclosed areas.
    boost::optional< Line > bisector;

    std::array< TailData, Endpoint::NumEndpoints > tailData;
};

TailMaker::TailMaker(
                const Stroke& stroke,
                const PreserveInterval& preserveMid,
                const Coll& coll,
                const BlendDrawings& bd )
    : _imp( std::make_unique< Imp >( stroke, preserveMid, coll, bd ) )
{
}

TailMaker::~TailMaker()
{}

UniqueStroke TailMaker::result()
{
    return _imp->result();
}

} // tails
} // mashup
