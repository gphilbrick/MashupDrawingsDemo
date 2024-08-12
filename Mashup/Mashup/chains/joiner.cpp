#include <chains/joiner.h>

#include <blenddrawings.h>
#include <blendoptions.h>
#include <chains/nextstep.h>
#include <drawings.h>
#include <strokeback.h>
#include <strokepoly.h>
#include <strokesegcollider.h>
#include <topology/crossing.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/math/curveutility.h>
#include <Core/model/curveback.h>
#include <Core/model/polyline.h>
#include <Core/model/stroketools.h>

#include <Core/utility/mathutility.h>

#include <boost/math/constants/constants.hpp>

#include <map>

namespace mashup {
namespace chains {

using Crossing = topology::Crossing;
using Pos = core::model::Pos;
using Stub = Substroke;
using StubPair = std::pair< Stub, Stub >;
using StubPairComp = std::function< bool( const StubPair&, const StubPair& ) >;

/// For a given pair of 'Stub's, a tentative pair of cutoff T values
/// that might yield a good joint.
using CutTPair = std::pair< double, double >;
using CutTPairs = std::vector< CutTPair >;
/// Used for generating 'CutTPair'.
using CutIJPair = std::pair< size_t, size_t >;
using CutIJPairs = std::vector< CutIJPair >;

namespace {

/// Return whether it would look good visually to make a joint connecting
/// the back of 'stub' to the front of 'stub'.
bool selfJoinWouldLookFine( const Stub& stub )
{
    // oriented to point outside of 'stub'
    const auto startDir = stub.endDirNormalized( false ) * -1.;
    const auto endDir = stub.endDirNormalized( true );

    // If the two endpoints are almost exactly _behind_ each other relative to their
    // normals, then a self-joint won't register well visually.

    const auto normalsApproxOpposite = []( const Pos& normA, const Pos& normB )
    {
        constexpr double maxRadiansAllowed = boost::math::constants::pi< double >() * 0.9;
        static const double minDotAllowed = std::cos( maxRadiansAllowed );
        return Pos::dot( normA, normB ) < minDotAllowed;
    };

    if( normalsApproxOpposite( startDir, endDir ) ) {
        // Is one endpoint "behind" the other?
        const auto startPos = stub.endpoint( false );
        const auto endPos = stub.endpoint( true );
        auto startToEnd = endPos - startPos;
        startToEnd.normalize();
        return !normalsApproxOpposite( startDir, startToEnd );
    } else {
        return true;
    }
}

bool compareStubPairs( const StubPair& a, const StubPair& b )
{
    if( a.first == b.first ) {
        return Stub::compare_standard( a.second, b.second );
    } else {
        return Stub::compare_standard( a.first, b.first );
    }
}

/// If we placed a circle of radius 'rad' at the end of 'stub', at what
/// approximate T value (no earlier than current start of 'stub') would the spine of 'stub'
/// enter that circle?
double eraseCircleFromEnd( const Stub& stub, double rad )
{
    const auto curve = stub.stroke->curve().extractCurveForTInterval( stub.t );    
    const auto tCurve = core::math::eraseCircleT( *curve, rad, false );
    return core::mathUtility::lerp( stub.t[ 0 ], stub.t[ 1 ], tCurve );
}

/// Produce a 'Joint' connecting the end of 'a' with the start of 'b' ('a' and 'b' are not 'Stub's, but
/// are co-aligned).
Joint smoothJoint( const Substroke& a, const Substroke& b )
{        
    auto posCurve = core::math::smoothJoint( a.endpoint( true ),
                                      a.endDirNormalized( true ),
                                      b.endpoint( false ),
                                      b.endDirNormalized( false ) );
    auto widthCurve = core::model::linearWidthCurve( a.endWidth( true ), b.endWidth( false ) );
    return core::model::strokeFromPosAndWidth( std::move( posCurve ), std::move( widthCurve ) );
}

/// A mapping from original 'Stub' of 'cross' to a "pretrimmed" version of said 'Stub'.
using Pretrimming = std::map< Stub, Stub, Stub::CompFunc >;

/// Information about one of the not-yet-connected 'Stub's ('stubOrig') in 'cross'.
struct StubData
{
    using CutTRange = std::pair< double, double >;

    StubData()
        : connected( false )
        , midT_orig( 0. )
    {}

    StubData( const Stub& stub, const BlendDrawings& bd )
        : connected( false )
        , midT_orig( core::mathUtility::lerp( stub.t[ 0 ], stub.t[ 1 ], 0.55 ) )
        , stubOrig( stub )
        , stubPretrimmed( stub )
    {
        const auto stubAsStroke = stub.asStroke();
        stubOrigPoly = StrokePoly( *stubAsStroke, bd.strokePolyLength( *stubAsStroke ) );
        findCutTRange( bd.options() );
    }

    /// 'pretrim' must fall within 'stubPretrimmed'.
    void setStubPretrimmed( const Stub& pretrim, const BlendOptions& opts )
    {
        if( stubPretrimmed.tIncreasing() ) {
            if( pretrim.t[ 0 ] < stubPretrimmed.t[ 0 ]
             || pretrim.t[ 1 ] > stubPretrimmed.t[ 1 ] ) {
                THROW_UNEXPECTED;
            }
        } else {
            if( pretrim.t[ 0 ] > stubPretrimmed.t[ 0 ]
             || pretrim.t[ 1 ] < stubPretrimmed.t[ 1 ] ) {
                THROW_UNEXPECTED;
            }
        }
        stubPretrimmed = pretrim;
        findCutTRange( opts );
    }

    void findCutTRange( const BlendOptions& opts )
    {
        const auto strongestT = strongestCutT( opts );
        const auto weakestT = stubPretrimmed.t[ 1 ];
        cutTRange = CutTRange{ strongestT, weakestT };
    }

    /// Return the first T value (moving along 'stubOrig' toward its 'Crossing') at which
    /// we are allowed to cut 'stubOrig'
    double strongestCutT( const BlendOptions& opts ) const
    {
        // For now let's not allow cutting off a 'stub' any earlier than 'midT_orig' is illegal.
        // This lets me determine joints at a per-'Crossing' level before generating
        // any 'Chains's.

        // There are risks here: if 'stubOrig' is very windy and goes in and out of
        // the circle we're "deleting" from its end here, results might look weird.
        const auto cutT = eraseCircleFromEnd( stubPretrimmed, opts.routing.jointRad );

        double ret = 0.;
        if( stubOrig.tIncreasing() ) {
            ret = std::max< double >( midT_orig, cutT );
        } else {
            ret = std::min< double >( midT_orig, cutT );
        }
        return ret;
    }        

    /// If the barrier 'barr' appeared, what would 'stubOrig' need to be pretrimmed to to accommodate?
    /// Return boost::none if no pretrimming would be required. Set 'tooMuch' to true if
    /// the amount of pretrimming would be illegal.
    boost::optional< Stub > wouldPretrimTo( const StrokePoly& barr, bool& tooMuch ) const
    {
        tooMuch = false;

        Stub ret = stubPretrimmed;

        for( const auto& sidePoly : barr.sides ) {
            const auto hitTs = stubOrigPoly.hitTs( sidePoly );
            for( const auto tNorm : hitTs ) {
                const auto tStub = core::mathUtility::lerp( stubOrig.t[ 0 ], stubOrig.t[ 1 ], tNorm );

                if( stubOrig.tIncreasing() ) {
                    if( tStub < midT_orig ) {
                        tooMuch = true;
                        return boost::none;
                    } else if( tStub < ret.t[ 1 ] ) {
                        ret.t[ 1 ] = tStub;
                    }
                } else {
                    if( tStub > midT_orig ) {
                        tooMuch = true;
                        return boost::none;
                    } else if( tStub > ret.t[ 1 ] ) {
                        ret.t[ 1 ] = tStub;
                    }
                }
            }
        }

        if( !( ret == stubPretrimmed ) ) {
            return ret;
        } else {
            return boost::none;
        }
    }

    bool hasBeenPretrimmed() const
    {
        return !( stubOrig == stubPretrimmed );
    }

    /// True if 'stubOrig' is a part of a finalized connection.
    bool connected;
    double midT_orig;
    Stub stubOrig;
    /// An attenuated version of 'stubOrig' (starts at same place, ends possibly earlier)
    /// based on accommodating established "barriers".
    Stub stubPretrimmed;
    /// The range of T values in 'stubOrig' in which we are allowed
    /// to make a cut for the purpose of attaching a joint, ordered
    /// (1) "strongest" cut-T (removes the most from 'stubOrig'
    /// (2) "weakest" cut-T (removes the least from 'stubOrig'
    CutTRange cutTRange;

    StrokePoly stubOrigPoly;
};

/// Information about a pair of (not equal) 'Stub's (say 'stubAOrig' and 'stubBOrig') from 'cross' that might be considered for joining.
struct PairData
{
    PairData()
        : pairChosen( false )
        , pretrimEffect( Stub::compare_standard )
    {}

    /// If it has been decided for certain that 'stubAOrig' and 'stubBOrig' are
    /// to be connected, then this is true.
    bool pairChosen;
    /// States how to make the step from 'stubAOrig' to 'stubBOrig' along a chain. If null,
    /// means it's not possible to do so.
    std::unique_ptr< NextStep > nextStep;
    /// If we decide on 'nextStep' as one of the connections that goes through,
    /// what is the pretrimming effect on 0 or more 'Stub's other than 'stubAOrig' and 'stubBOrig'.
    Pretrimming pretrimEffect;
};

} // unnamed

struct Joiner::Imp
{
    Imp( const topology::Crossing& c, const BlendDrawings& bd )
        : cross( c )
        , blendDrawings( bd )
        , collAB( bd.collAB() )
        , opts( bd.options() )
        , drawings( bd.drawings() )
        , stubData( Stub::compare_standard )
        , pairData( compareStubPairs )
    {
        findCutIJ();

        const auto& stubs = cross.stubs();
        for( const auto& stub : stubs ) {
            stubData[ stub ] = StubData{ stub, blendDrawings };
        }

        for( size_t i = 0; i < stubs.size(); i++ ) {
            const auto& stubA = stubs[ i ];
            for( size_t j = i + 1; j < stubs.size(); j++ ) {
                const auto& stubB = stubs[ j ];
                pairData[ StubPair{ stubA, stubB } ];
                pairData[ StubPair{ stubB, stubA } ];
            }
        }

        updatePairsConnectibility();
    }

    /// Look at all 'Stub' pairs which we haven't already decided to connect and determine
    /// whether it is possible to connect them.
    void updatePairsConnectibility()
    {
        const auto& stubs = cross.stubs();
        for( size_t a = 0; a < stubs.size(); a++ ) {
            const auto& stubA = stubs[ a ];
            if( stubData.find( stubA )->second.connected ) {
                continue;
            }
            for( size_t b = a + 1; b < stubs.size(); b++ ) {
                const auto& stubB = stubs[ b ];
                if( stubData.find( stubB )->second.connected ) {
                    continue;
                }
                auto& pairAB = pairData.find( StubPair{ stubA, stubB } )->second;
                if( pairAB.pairChosen ) {
                    // We've already finalized that 'stubA' and 'stubB' are joined; nothing to do.
                    continue;
                }

                pairAB.nextStep = tentativeJoin( stubA, stubB, pairAB.pretrimEffect );

                // Store mirrored info in BA data.
                auto& pairBA = pairData.find( StubPair{ stubB, stubA } )->second;
                if( pairAB.nextStep ) {
                    pairBA.pretrimEffect = pairAB.pretrimEffect;
                    pairBA.nextStep = pairAB.nextStep->reverse( stubA );
                } else {
                    pairBA.nextStep = nullptr;
                    pairBA.pretrimEffect.clear();
                }
            }
        }
    }

    /// Return whether 'next' manages not to hit anything it's not supposed to.
    /// If return value is true, store in 'pretrim' the pretrimming effect that it would have
    /// on uninvolved 'Stub's if 'next' were accepted.
    /// 'prev' is an original 'Stub' of 'cross'.
    bool validTentativeConnection( const Stub& prev, const NextStep& next, Pretrimming& pretrim ) const
    {        
        pretrim.clear();

        // Middle part of 'next' (excluding the (pretrimmed) from-'cross' 'Stub's).
        auto nextMidStroke = next.midStroke();
        const auto midPoly = StrokePoly{ *nextMidStroke, blendDrawings.strokePolyLength( *nextMidStroke ) };

        // PRETRIMMING
        // For each unconnected 'Stub' in 'cross' (other than the 'Stub's we're considering connecting),
        // find a new trim-T based on using two sides of midPoly. Make sure that no too-severe
        // pretrim would result (abandon 'next' and return false if so).
        const auto stubs = cross.stubs();
        for( const auto& stub : stubs ) {
            if( stub == prev || stub == next.next.reverse() ) {
                continue;
            }

            const auto& sData = stubData.find( stub )->second;
            if( sData.connected ) {
                // 'stub' is already part of a finalized connection so pretrimming is moot for it.
                continue;
            }

            bool tooMuchPretrim = false;
            const auto res = sData.wouldPretrimTo( midPoly, tooMuchPretrim );
            if( tooMuchPretrim ) {
                return false;
            } else if( res.is_initialized() ) {
                pretrim[ stub ] = *res;
            }
        }

        // BARRIERS
        // Make sure that 'next' doesn't touch any of 'barriers'.
        for( const auto& barr : barriers ) {
            for( const auto& midPolySide : midPoly.sides ) {
                if( barr.hitsAtAll( midPolySide ) ) {
                    return false;
                }
            }
        }

        // COLLIDER
        // Make sure that 'next' doesn't touch anything it should not in the original A/B drawings.
        {
            const auto disqualifyingCollision = [ & ]( StrokeHandle stroke, double tStroke )
            {
                if( opts.preserveDrawing && *opts.preserveDrawing == drawings.whichDrawing( stroke ) ) {
                    return true;
                }

                // Ignore collisions with 'Stroke' pieces involved in 'cross' (except
                // preserve-drawing collisions).
                if( cross.isPartOf( stroke, tStroke ) ) {
                    return false;
                } else {
                    return true;
                }
            };

            for( const auto& midPolySide : midPoly.sides ) {
                if( collAB.hitsAnythingPassing( midPolySide, disqualifyingCollision ) ) {
                    return false;
                }
            }
        }

        return true;
    }

    /// Return, in order, the positions in the 2D space of options for
    /// cutting off 'stubA' and 'stubB' such that we find as early as possible
    /// a valid joint involving as much cutting (in terms of T) as possible.
    CutTPairs cutPairs( const Stub& stubA, const Stub& stubB ) const
    {
        const auto& aRange = stubData.find( stubA )->second.cutTRange;
        const auto& bRange = stubData.find( stubB )->second.cutTRange;

        CutTPairs ret;
        for( const auto& ij : cutIJ ) {
            const auto f_i = F_FROM_I( ij.first, numSteps );
            const auto f_j = F_FROM_I( ij.second, numSteps );

            const auto cutT_a = core::mathUtility::lerp( aRange.first, aRange.second, f_i );
            const auto cutT_b = core::mathUtility::lerp( bRange.first, bRange.second, f_j );
            ret.push_back( { cutT_a, cutT_b } );
        }
        return ret;
    }

    /// 'stubA' and 'stubB' are original (untrimmed) stubs from 'cross', neither of which has a (finalized) connection yet.
    /// If return value is non-null, 'storePretrim' says what effect of adopting return value as a connection would be.
    std::unique_ptr< NextStep > tentativeJoin( const Stub& stubA, const Stub& stubB, Pretrimming& storePretrim ) const
    {
        storePretrim.clear();

        const bool eitherPretrimmed =
                stubData.find( stubA )->second.hasBeenPretrimmed()
             || stubData.find( stubB )->second.hasBeenPretrimmed();

        // SPECIAL CASE: originally connected
        if( cross.originallyConnected( stubA, stubB ) ) {
            // For now, let the only connection between these two be the original full 'Stroke' interval itself (no joints).
            if( eitherPretrimmed ) {
                return nullptr;
            }
            // 'a' and 'b' can be represented as a single 'Stroke' interval; we don't
            // need to find a joint between them.
            auto ret = NextStep::singleIntervalCase( stubA, stubB );
            if( validTentativeConnection( stubA, *ret, storePretrim ) ) {
                return ret;
            } else {
                return nullptr;
            }
        }

        // SPECIAL CASE: same 'Substroke'
        // 'stubA' and 'stubB' actually represent the same 'Substroke' (but are not originally connected).
        // This case requires special treatment if for no other reason than that the 0.55 mid-T trimming
        // cutoff assumes that stub pairs do not represent the same 'Substroke'.
        if( stubA == stubB.reverse() ) {
            if( eitherPretrimmed ) {
                // Same as with previous special case: there is only one kind of outcome for this pair I care about
                // making possible right now.
                return nullptr;
            }
            if( !selfJoinWouldLookFine( stubA ) ) {
                // The joint would probably look bad. (Most likely 'stubA' is or resembles
                // a line segment, something that shouldn't have its ends joined to each
                // other).
                return nullptr;
            }

            auto ret = std::make_unique< NextStep >();
            ret->prevTrimmed = stubA;
            ret->nextTrimmed = stubA;
            ret->next = stubA;
            ret->joint = smoothJoint( stubA, stubA );
            if( validTentativeConnection( stubA, *ret, storePretrim ) ) {
                return ret;
            } else {
                return nullptr;
            }
        }

        auto ret = std::make_unique< NextStep >();
        ret->next = stubB.reverse();

        // Search the 2D space of places to cut off 'stubA' (from 'cutT_a' to end of 'stubA')
        // and places to cut off 'stubB' (from 'cutT_b' to end of 'stubB') until we produce
        // a joint that passes.
        //
        // Try to find the first valid joint involving the most cutting.

        const auto cutABs = cutPairs( stubA, stubB );
        for( const auto& cutAB : cutABs ) {
            ret->prevTrimmed = Stub( *stubA.stroke, stubA.t[ 0 ], cutAB.first );
            // note the reversal here: 'nextTrimmed' heads _out_ of the crossing
            ret->nextTrimmed = Substroke( *stubB.stroke, cutAB.second, stubB.t[ 0 ] );
            ret->joint = smoothJoint( ret->prevTrimmed, ret->nextTrimmed );
            if( validTentativeConnection( stubA, *ret, storePretrim ) ) {
                return ret;
            }
        }

        // Couldn't find anything that worked.
        return nullptr;
    }

    void findCutIJ()
    {
        if( cutIJ.size() == 0 ) {
            static_assert( numSteps > 0, "Invalid numSteps" );
            for( size_t i = 0; i < numSteps; i++ ) {
                for( size_t j = 0; j < numSteps; j++ ) {
                    cutIJ.push_back( { i, j } );
                }
            }
            std::sort(
                cutIJ.begin(),
                cutIJ.end(),
                []( const CutIJPair& a, const CutIJPair& b )
            {
                // A smaller i/j means a bigger cut, which we want.
                return ( a.first + a.second ) < ( b.first + b.second );
            } );
        }
    }

    /// 'stubA' and 'stubB' are original 'Stub's of 'cross'.
    bool canJoin( const Stub& stubA, const Stub& stubB ) const
    {
        const auto& pData = pairData.find( StubPair{ stubA, stubB } )->second;
        return pData.nextStep != nullptr;
    }

    /// Finalize the (already planned out) connection from 'stubA' to 'stubB'.
    /// 'stubA' and 'stubB' are original 'Stub's of 'cross'.
    std::unique_ptr< NextStep > join( const Stub& stubA, const Stub& stubB )
    {
        auto& abData = pairData.find( StubPair{ stubA, stubB } )->second;
        auto& baData = pairData.find( StubPair{ stubB, stubA } )->second;
        if( abData.nextStep == nullptr || baData.nextStep == nullptr ) {
            THROW_RUNTIME( "This pair is not connectible" );
        }
        if( abData.pairChosen || baData.pairChosen ) {
            THROW_UNEXPECTED;
        }

        // Mark as finalized.
        abData.pairChosen = true;
        baData.pairChosen = true;
        stubData.find( stubA )->second.connected = true;
        stubData.find( stubB )->second.connected = true;

        // Trim other 'Stub's to make room for the connection.
        for( const auto& pair : abData.pretrimEffect ) {
            const auto& stub = pair.first;
            auto& data = stubData.find( stub )->second;
            if( data.connected ) {
                THROW_UNEXPECTED;
            }
            data.setStubPretrimmed( pair.second, opts );
        }

        // Create a barrier representing this new connection.
        {
            auto asStroke = abData.nextStep->asStroke();
            barriers.push_back( StrokePoly{ *asStroke, blendDrawings.strokePolyLength( *asStroke ) } );
        }

        updatePairsConnectibility();

        // Clear out the fields of 'pData' we don't need anymore.
        abData.pretrimEffect.clear();
        baData.pretrimEffect.clear();
        baData.nextStep = nullptr;
        return std::move( abData.nextStep );
    }

    /// Used in 'cutPairs'
    static std::vector< std::pair< size_t, size_t > > cutIJ;
    /// In the 1D space of cut possibilities for one 'Stub', how many positions
    /// should we try.
    static const size_t numSteps = 5;

    const Crossing& cross;
    const BlendDrawings& blendDrawings;
    const StrokeSegCollider& collAB;
    const BlendOptions& opts;
    const Drawings& drawings;

    std::map< Stub, StubData, Stub::CompFunc > stubData;
    std::map< StubPair, PairData, StubPairComp > pairData;    
    /// 'barriers' created by finalized connections.
    std::vector< StrokePoly > barriers;
};

std::vector< std::pair< size_t, size_t > > Joiner::Imp::cutIJ;

Joiner::Joiner( const topology::Crossing& c, const BlendDrawings& bd )
    : _imp( std::make_unique< Imp >( c, bd ) )
{
}

Joiner::~Joiner()
{}

bool Joiner::canJoin( const Stub& a, const Stub& b ) const
{
    return _imp->canJoin( a, b );
}

std::unique_ptr< NextStep > Joiner::join( const Stub& a, const Stub& b )
{
    return _imp->join( a, b );
}

Stub Joiner::pretrimmed( const Stub& stub ) const
{
    return _imp->stubData.find( stub )->second.stubPretrimmed;
}

} // chains
} // mashup
