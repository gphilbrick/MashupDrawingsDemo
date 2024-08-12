#include <chains/router.h>

#include <blenddrawings.h>
#include <blendoptions.h>
#include <chains/joiner.h>
#include <chains/nextstep.h>
#include <drawings.h>
#include <randombinary.h>
#include <strokeback.h>
#include <strokesegcollider.h>
#include <substroke.h>
#include <topology/crossing.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/model/lineback.h>
#include <Core/model/stroketools.h>

#include <Core/utility/mathutility.h>

#include <algorithm>
#include <map>
#include <set>

namespace mashup {
namespace chains {

using Pos = core::model::Pos;
using Seg = core::model::Seg;
using Stub = Substroke;

namespace {

/// Given a 'prob' in [0,1] and a 'gamma' in [0,1], push 'prob'
/// away from 0.5 toward 1. or 0. depending on which side of 0.5
/// it's on.
double sharpenProbability( double prob, double gamma )
{
    if( gamma == 0. ) {
        if( prob == 0.5 ) {
            return prob;
        } else if( prob < 0.5 ) {
            return 0.;
        } else {
            return 1.;
        }
    } else {
        const double sign = prob > 0.5 ? 1. : -1.;
        return std::clamp(
            0.5 + std::pow( 2. * std::abs( prob - 0.5 ), gamma ) / 2. * sign,
            0.,
            1. );
    }
}

/// Data for some 'stub'.
struct StubData
{
    /// What is the >=0 weight of 'stub' relative to other stubs at the same
    /// 'Crossing'.
    double weight = 0.;

    /// If 'stub' is an (untrimmed) part of a chain, how do we take the next
    /// step on that chain.
    std::unique_ptr< NextStep > next;

    /// If 'stub' is the end of a chain ('next' is nullptr), and some "pretrimming"
    /// needs to be applied to 'stub', then this is that pretrimmed version.
    boost::optional< Stub > stubPretrimmed;

    DrawingID drawingID = DrawingID::NumDrawings;
};

} // unnamed

using VoteWithWeight = std::pair< Stub, double >;
using Votes = std::vector< VoteWithWeight >;

struct Router::Imp
{
    Imp( const Crossing& c, const BlendDrawings& bd, RandomBinary& rand )
        : stubToData( Stub::compare_standard )
        , blendDrawings( bd )
        , opts( bd.options() )
        , collAB( bd.collAB() )
        , drawings( bd.drawings() )
    {
        const auto& stubs = c.stubs();
        for( const auto& s : stubs ) {
            stubToData[ s ] = {};
        }
        init( c, rand );
    }

    /// Return options that 'stubA' can legally connect to, in descending order of preference.
    Votes connectToVotes( const Substroke& stubA, const Joiner& joiner, const boost::optional< Substroke >& ignore ) const
    {
        Votes ret;
        for( const auto& pair : stubToData ) {
            const auto& stubB = pair.first;
            if( stubA == stubB ) {
                continue;
            }
            if( ignore && ignore.value() == stubB ) {
                continue;
            }                        
            const auto& stubBData = pair.second;
            if( stubBData.next ) {
                continue;
            }
            if( stubBData.drawingID == opts.preserveDrawing ) {
                continue;
            }

            if( joiner.canJoin( stubA, stubB ) ) {
                ret.push_back( { stubB, stubBData.weight } );
            }
        }

        std::sort(
            ret.begin(),
            ret.end(),
            [ & ]( const VoteWithWeight& a, const VoteWithWeight& b )
            {
                const auto& aWeight = a.second;
                const auto& bWeight = b.second;
                if( aWeight == bWeight ) {
                    const auto& aStub = a.first;
                    const auto& bStub = b.first;
                    return deterministicSort_weightsAreSame( aStub, bStub );
                } else {
                    return aWeight > bWeight;
                }
            } );

        return ret;
    }

    std::unique_ptr< NextStep > next( const Substroke& prev, boost::optional< Substroke >& prevPretrimmed ) const
    {
        prevPretrimmed = boost::none;
        const auto it = stubToData.find( prev );
        if( it == stubToData.end() ) {
            THROW_UNEXPECTED;
        } else {
            const auto& stubData = it->second;
            if( stubData.next ) {
                return stubData.next->clone();
            } else {
                prevPretrimmed = stubData.stubPretrimmed;
                return nullptr;
            }
        }
    }

    /// Put higher-weighted before lower-weighted. When weights are
    /// the same, apply some deterministic ordering.
    bool deterministicSort( const Stub& a, const Stub& b ) const
    {
        const auto aWeight = stubToData.find( a )->second.weight;
        const auto bWeight = stubToData.find( b )->second.weight;
        if( aWeight == bWeight ) {
            return deterministicSort_weightsAreSame( a, b );
        } else {
            return aWeight > bWeight;
        }
    }

    /// If 'a' and 'b' have same 'Stroke', their T-intervals must differ.
    bool deterministicSort_weightsAreSame( const Stub& a, const Stub& b ) const
    {
        // This is where we ensure determinism.
        if( a.stroke == b.stroke ) {
            // Assuming we won't have two equal-T-same-'Stroke' 'Stub's.
            if( a.t[ 0 ] == b.t[ 0 ] ) {
                return a.t[ 1 ] < b.t[ 1 ];
            } else {
                return a.t[ 0 ] < b.t[ 0 ];
            }
        } else {
            const auto dIdx_a = drawings.whichDrawing( a.stroke );
            const auto dIdx_b = drawings.whichDrawing( b.stroke );
            if( dIdx_a == dIdx_b ) {
                const auto& drawing = drawings[ dIdx_a ];
                const auto aIndex = drawing.index( a.stroke );
                const auto bIndex = drawing.index( b.stroke );
                return aIndex < bIndex;
            } else {
                return dIdx_a < dIdx_b;
            }
        }
    }

    void init( const Crossing& cross, RandomBinary& rand )
    {
        Joiner joiner( cross, blendDrawings );

        // Score everything and assign DrawingID
        std::vector< Stub > stubsSorted;
        for( auto& pair : stubToData ) {
            const auto& key = pair.first;
            auto& data = pair.second;
            data.weight = opts.routing.wFunctor->weight( *key.stroke, key.t[ 1 ], blendDrawings );
            data.drawingID = drawings.whichDrawing( key.stroke );
            stubsSorted.push_back( key );
        }

        std::sort(
            stubsSorted.begin(),
            stubsSorted.end(),
            [ & ]( const Stub& a, const Stub& b )
            {
                return deterministicSort( a, b );
            } );

        // Assign connections to highest-weight stubs first.
        for( const auto& stubA : stubsSorted ) {
            auto& stubAData = stubToData.find( stubA )->second;

            // Don't bother joining to-preserve-drawing stubs
            if( opts.preserveDrawing == stubAData.drawingID ) {
                continue;
            }

            if( stubAData.next ) {
                // Already linked.
                continue;
            }

            const auto oldConnection = cross.originalConnection( stubA ); // Did 'stub' connect to something (from this Xing) in its original drawing?
            boost::optional< VoteWithWeight > oldConnVote;
            if( oldConnection ) {
                // Old connection exists, but is it usable?
                const auto& data = stubToData.find( *oldConnection )->second;
                if( !data.next && joiner.canJoin( stubA, *oldConnection ) ) {
                    // We are allowed to use the "old connection".
                    oldConnVote = VoteWithWeight{ *oldConnection, data.weight };
                }
            }
            const auto otherVotes = connectToVotes( stubA, joiner, oldConnection );

            // Come up w/ top two votes.
            const VoteWithWeight* vote1 = oldConnVote
                    ? &oldConnVote.value()
                    : ( otherVotes.size() ? &otherVotes[ 0 ] : nullptr );
            const VoteWithWeight* vote2 = oldConnVote
                    ? ( otherVotes.size() ? &otherVotes[ 0 ] : nullptr )
                    : ( otherVotes.size() > 1 ? &otherVotes[ 1 ] : nullptr );

            const Stub* stubB = nullptr;
            if( vote1 && vote2 ) {

                // What are the [0,1] odds of randomly choosing 'vote2' instead of 'vote1'?
                double prob = 0.;
                constexpr double fiftyFifty = 0.5;

                const auto weight1 = vote1->second;
                const auto weight2 = vote2->second;
                const auto weightSum = weight1 + weight2;
                if( core::mathUtility::closeEnoughToZero( weightSum ) ) {
                    // edge case: treat the two votes as being about equal.
                    prob = fiftyFifty;
                } else {
                    prob = sharpenProbability( weight2 / weightSum, opts.routing.flipVoteGamma );
                }
                stubB = rand.yes( prob ) ? &vote2->first : &vote1->first;
            } else if( vote1 ) {
                stubB = &vote1->first;
            }

            if( stubB ) {
                stubAData.next = joiner.join( stubA, *stubB );
                if( !stubAData.next ) {
                    THROW_UNEXPECTED;
                }
                auto& stubBData = stubToData.find( *stubB )->second;
                stubBData.next = stubAData.next->reverse( stubA );
            } else {
                // 'stubA' is the end of a chain. Do we need pretrimming information so that this chain ends
                // as early as it's supposed to (and doesn't overrun connections from same 'Crossing')?
                stubAData.stubPretrimmed = joiner.pretrimmed( stubA );
            }
        }
    }

    std::map< Stub, StubData, Stub::CompFunc > stubToData;
    const BlendDrawings& blendDrawings;
    const BlendOptions& opts;
    const StrokeSegCollider& collAB;
    const Drawings& drawings;
};

Router::Router( const Crossing& c,
               const BlendDrawings& bd,
               RandomBinary& r )
    : _imp( std::make_unique< Imp >( c, bd, r ) )
{
}

Router::~Router()
{
}

std::unique_ptr< NextStep > Router::next( const Substroke& prev, boost::optional< Substroke >& prevPretrimmed ) const
{
    return _imp->next( prev, prevPretrimmed );
}

} // chains
} // mashup
