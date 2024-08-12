#include <chains/chainbuilder.h>

#include <blenddrawings.h>
#include <blendoptions.h>
#include <chains/nextstep.h>
#include <chains/router.h>
#include <drawings.h>
#include <randombinary.h>
#include <strokeback.h>
#include <strokesegcollider.h>
#include <topology/topology.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/view/progressbar.h>

#include <map>

namespace mashup {
namespace chains {

using Topol = topology::Topology;

namespace {

/// Make a shortened 'toTrim' that starts no earlier ('start'=true) or ends
/// no later ('start'=false) than 'trimWith'.
///
/// 'toTrim' and 'trimWith' must be same-direction intervals on the same 'Stroke'.
void trim( Substroke& toTrim, const Substroke& trimWith, bool start )
{
    if( toTrim.stroke != trimWith.stroke ) {
        THROW_RUNTIME( "trimming with bad trimmer--not same stroke" );
    }
    if( toTrim.tIncreasing() != trimWith.tIncreasing() ){
        THROW_RUNTIME( "trimming with bad trimmer--not same tIncreasing" );
    }
    toTrim.nonFlippingTrim( ( start ? trimWith.t[ 0 ] : trimWith.t[ 1 ] ), start );
}

} // unnamed

struct ChainBuilder::Imp
{
    Imp( const Topol& t, const BlendDrawings& bd, core::view::ProgressBar* pb )
        : topol( t )
        , blendDrawings( bd )
        , opts( bd.options() )
        , collAB( bd.collAB() )
        , drawings( bd.drawings() )
        , progBar( pb )
    {
        const auto crossings = topol.crossings();

        const bool doProg = progBar && crossings.size();
        if( doProg ) {
            progBar->startOnlyStage( "Setting up routers", static_cast< int >( crossings.size() ) );
        }

        int numDone = 0;
        RandomBinary rand;
        for( const auto* const c : crossings ) {
            auto router = std::make_unique< Router >( *c, blendDrawings, rand );
            crossingToRouter[ c ] = std::move( router );
            if( doProg ) {
                progBar->update( ++numDone );
            }
        }
    }

    /// If 'ss' ends at a crossing, return the 'Router' representing that
    /// crossing; else return nullptr. Only call after calling 'buildRouters'.
    Router* findRouter( const Substroke& ss ) const
    {
        const auto* const crossing = topol.findCrossing( ss );
        if( !crossing ) {
            return nullptr;
        }

        const auto it = crossingToRouter.find( crossing );
        if( it == crossingToRouter.end() ) {
            THROW_UNEXPECTED;
        }

        return it->second.get();
    }

    std::vector< Substroke > startSubstrokes() const
    {
        const auto unoccluded = topol.unoccludedSubstrokes( blendDrawings.drawings() );
        if( opts.preserveDrawing ) {
            const auto toPreserveDID = *opts.preserveDrawing;
            // Only use as starting 'Substroke's those that
            // are not from the drawing to preserve.
            std::vector< Substroke > ret;
            for( const auto& ss : unoccluded ) {
                if( drawings.whichDrawing( ss.stroke ) != toPreserveDID ) {
                    ret.push_back( ss );
                }
            }
            return ret;
        } else {
            return unoccluded;
        }
    }

    UniqueChains chains()
    {
        UniqueChains ret;

        const auto substrokesToDo = startSubstrokes();

        const bool doProg = progBar && substrokesToDo.size();
        if( doProg ) {
            progBar->startOnlyStage( "Building chains", static_cast< int >( substrokesToDo.size() ) );
        }

        // Track which 'Substroke's we've already seen, with directional invariance
        // ([i,j] on stroke S is identical to [j,i] on stroke S).
        const auto compSubstrokes = []( const Substroke& a, const Substroke& b )
        {
            if( a.stroke == b.stroke ) {
                // Treat 'Substrokes' as directionally invariant.
                const auto aMM = std::minmax( a.t[ 0 ], a.t[ 1 ] );
                const auto bMM = std::minmax( b.t[ 0 ], b.t[ 1 ] );
                return aMM < bMM;
            } else {
                return a.stroke < b.stroke;
            }
        };
        std::set< Substroke, decltype( compSubstrokes ) > substrokesHandled( compSubstrokes );

        int progBarItemsDone = 0;
        for( const auto& substroke : substrokesToDo ) {
            if( doProg ) {
                progBar->update( progBarItemsDone++ );
            }

            // Already done?
            if( substrokesHandled.find( substroke ) != substrokesHandled.end() ) {
                continue;
            } else {
                substrokesHandled.emplace( substroke );
            }

            /// Return a 'Chain' that starts with 'startFrom', which is one of 'substrokesToDo' or
            /// a reverse() of one.
            const auto buildChain = [ & ]( const Substroke& startFrom ) -> std::unique_ptr< Chain >            
            {
                auto ret = std::make_unique< Chain >();
                ret->substrokes.push_back( startFrom );
                // One of 'substrokesToDo' or reverse() of one of these.
                auto curSubstroke = startFrom;

                while( true ) {
                    // Find the crossing/router associated with end of 'substroke'
                    const auto* const router = findRouter( curSubstroke );
                    if( router ) {
                        boost::optional< Substroke > curSSPretrimmed; // used only if 'next' is nullptr
                        auto next = router->next( curSubstroke, curSSPretrimmed );
                        if( next ) {
                            const auto& nextSS_untrimmed = next->next;
                            const auto& nextSS_trimmed = next->nextTrimmed;
                            const auto& prevSS_trimmed = next->prevTrimmed;
                            auto& joint = next->joint;

                            // If necessary trim off some of the back end of previous substroke to
                            // make room for joint
                            trim( ret->substrokes.back(), prevSS_trimmed, false );

                            if( substrokesHandled.find( nextSS_untrimmed ) == substrokesHandled.end() ) {
                                ret->substrokes.push_back( nextSS_trimmed );
                                ret->joints.push_back( joint ? std::move( joint ) : nullptr );
                                substrokesHandled.emplace( nextSS_untrimmed );
                                curSubstroke = nextSS_untrimmed;
                            } else {
                                // Our next step is something already handled. The only way that's legal is
                                // if 'nextSS_untrimmed' equals 'startFrom' and we're dealing with a closed
                                // chain, _and if this is the first time we're seeing said closed chain_.
                                if( startFrom == nextSS_untrimmed ) {
                                    if( joint ) {
                                        // Propagate trim to the front of 'ret'.
                                        trim( ret->substrokes.front(), nextSS_trimmed, true );
                                        ret->joints.push_back( std::move( joint ) );
                                    } else {
                                        THROW_RUNTIME( "Null joint unexpected when closing a chain." );
                                    }
                                    ret->closed = true;
                                    break;
                                } else {
                                    // This could mean a 'Router' bug (routing two different stubs to the same stub?).
                                    THROW_UNEXPECTED;
                                }
                            }
                        } else {
                            // Path is cut off here.
                            if( !curSSPretrimmed ) {
                                // This should be set to let us know how early the chain should be cut off so as not
                                // to intersect connections from the same 'Crossing'.
                                THROW_UNEXPECTED;
                            }
                            // Possibly cut off early by unrelated joints at 'router'.
                            trim( ret->substrokes.back(), *curSSPretrimmed, false );
                            break;
                        }
                    } else {
                        // Chain ends "naturally" here (with T=0/1 of some Stroke).
                        break;
                    }
                }
                return ret;
            };

            UniqueChain chain;
            {
                const auto numHandledBefore = substrokesHandled.size();

                // Normally, we'll build two halves and then stitch 'chain' together
                // from these halves.

                auto chain1 = buildChain( substroke );
                if( chain1->closed ) {
                    // No need for second half in this case, obviously.
                    chain = std::move( chain1 );
                } else {
                    const auto chain2 = buildChain( substroke.reverse() );
                    const auto numHandledThisChain = substrokesHandled.size() - numHandledBefore;
                    // Sanity check
                    if( numHandledThisChain != chain1->substrokes.size() + chain2->substrokes.size() - 2 ) {
                        THROW_UNEXPECTED;
                    }
                    chain = Chain::checkAndCombineHalves( *chain1, *chain2 );
                }
            }

            if( !chain ) {
                THROW_RUNTIME( "Failed to build chain" );
            }
            // Catch bugs with stitching and joints
            if( chain->hasBadJoint( 1. ) ) {
                THROW_RUNTIME( "Made chain that will bad-stitch" );
            }

            ret.push_back( std::move( chain ) );
        }

        return ret;
    }

    const Topol& topol;
    const BlendDrawings& blendDrawings;
    const BlendOptions& opts;
    const StrokeSegCollider& collAB;
    const Drawings& drawings;

    /// Associate a 'Router' with each 'Crossing' in 'topol'.
    std::map< const topology::Crossing*, std::unique_ptr< Router > > crossingToRouter;

    core::view::ProgressBar* const progBar;
};

ChainBuilder::ChainBuilder( const Topol& t, const BlendDrawings& bd, core::view::ProgressBar* progBar )
    : _imp( std::make_unique< Imp >( t, bd, progBar ) )
{
}

ChainBuilder::~ChainBuilder()
{
}

UniqueChains ChainBuilder::chains()
{
    return _imp->chains();
}

} // chains
} // mashup
