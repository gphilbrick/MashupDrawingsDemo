#include <blenddrawings.h>

#include <blendoptions.h>
#include <chains/chainbuilder.h>
#include <endpoint.h>
#include <samedrawinghits.h>
#include <strokeback.h>
#include <strokepoly.h>
#include <strokesegcollider.h>
#include <tails/tailmaker.h>
#include <topology/findtopology.h>
#include <topology/topology.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/math/curveutility.h>
#include <Core/model/curveback.h>
#include <Core/model/stroke.h>
#include <Core/model/stroketools.h>
#include <Core/view/progressbar.h>

#include <Core/utility/boundinginterval.h>
#include <Core/utility/casts.h>

#include <array>
#include <map>

namespace mashup {

using BoundingInterval = core::BoundingIntervald;
using Chain = chains::Chain;
using Pos = core::model::Pos;
using PreserveInterval = tails::TailMaker::PreserveInterval;
using UniqueStroke = std::unique_ptr< Stroke >;
using UniqueStrokes = std::vector< UniqueStroke >;

namespace {

core::model::BoundingBox bounds( const Drawings& d )
{        
    core::model::BoundingBox ret;
    d[ DrawingID::DrawingA ].forEach( [ &ret ]( const Stroke& s )
    {
        ret.growToContain( s.boundingBox() );
    } );
    d[ DrawingID::DrawingB ].forEach( [ &ret ]( const Stroke& s )
    {
        ret.growToContain( s.boundingBox() );
    } );
    return ret;
}

} // unnamed

struct BlendDrawings::Imp
{
    Imp( const BlendDrawings* p, const BlendOptions& opts, Drawings&& toOwn )
        : parent( p )
        , opts( opts )
        , drawings( std::move( toOwn ) )
        , collAB( bounds( drawings ) )
        , progBar( nullptr )
    {
        if( progBar ) {
            progBar->startOnlyStage( "Set up original-Strokes collider and same-drawing-hits" );
        }

        for( int d = 0; d < DrawingID::NumDrawings; d++ ) {
            drawings[ d ].forEach( [ & ]( const Stroke& s )
              {
                auto& strokePoly = sToPoly[ &s ];
                strokePoly = StrokePoly( s, strokePolyLength( s ) );
                collAB.addStroke( strokePoly );
              } );
        }
        collAB.sameDrawingHits( sameDrawingHits, drawings );
    }

    /// Return how many points the (spine of a) polyline-based 'Stroke' approximation of 's'
    /// should be.
    size_t strokePolyLength( const Stroke& s ) const
    {
        const auto& scenarioBounds = collAB.bounds();
        size_t numPoints_lengthBased = 0;
        {
            const size_t numPointsPerMaxDim = 160;
            numPoints_lengthBased = static_cast< size_t >(
                ( s.curve().cachedLength() / scenarioBounds.avgDim() ) * static_cast< double >( numPointsPerMaxDim ) );
        }
        const size_t numPoints_curvesBased = std::max< size_t >( 10, size_t( s.curve().numBezierCurves( false ) ) );

        const size_t numPoints = std::max< size_t >( numPoints_lengthBased, numPoints_curvesBased );
        return numPoints;
    }

    /// Given 'cComplex' (has at least one 'Substroke') representing some blend-stroke 'bs', store
    /// in 'storePretails' the version of 'bs' prior to having its ends converted to tails, and
    /// store in 'storePreserve' the interval of 'storePretails' that must not be converted to tails.
    void pretailsAndPreserve(
        const Chain& cComplex,
        UniqueStroke& storePretails,
        PreserveInterval& storePreserve ) const
    {
        storePretails = nullptr;
        storePreserve = boost::none;

        if( cComplex.substrokes.size() < 1 ) {
            THROW_UNEXPECTED;
        }

        const bool singleSSChain = cComplex.substrokes.size() == 1;

        // To determine tailable regions, we'll take each ending 'Substroke' (ESS) of 'cComplex',
        // treat it as oriented so that it ends ('Substroke' F = 1) at the end of 'cComplex', and
        // then pick a cut F value in [0,1] after which ESS should be considered tailable.

        // Keep in (0,1) to help avoid annoying edge cases where we cut off slightly more
        // than we want to.
        const double minCutF = 0.1;

        // These end-'Substroke's start inside 'cComplex' and at at 'cComplex''s endpoints.
        const std::array< Substroke, 2 > ess{
            cComplex.substrokes.front().reverse(),
            cComplex.substrokes.back() };
        // How much shall we preserve of (keep non-tailable of) each in 'ess'.
        // 1. means complete preservation, no tail.
        std::array< double, Endpoint::NumEndpoints > ess_cutF{ 1., 1. };
        {
            for( size_t i = 0; i < Endpoint::NumEndpoints; i++ ) {
                if( !cComplex.hasTail( i == Start ) ) {
                    continue;
                }

                auto& cutF = ess_cutF[ i ];
                cutF = minCutF;

                // 'ess[ i ]' has to terminate at a 'Crossing' or 'cComplex' would not
                // have a tail at end 'i'.
                const auto* const endpointCrossing = topol->findCrossing( ess[ i ] );
                if( !endpointCrossing ) {
                    THROW_RUNTIME( "Could not find 'Crossing' for this end of the chain." );
                }

                const auto* const stroke = ess[ i ].stroke;

                // Which 'Drawing' is our end-'Substroke' associated with?
                const auto dID = drawings.whichDrawing( stroke );
                if( dID == DrawingID::NumDrawings ) {
                    THROW_UNEXPECTED;
                }
                const auto& sdh = sameDrawingHits[ dID ];

                // Possibly increase 'cutF' to get past the most distal same-drawing
                // intersection.
                auto t_stroke = sdh.firstOrLastHit( ess[ i ], false, *endpointCrossing );
                if( t_stroke.is_initialized() ) {
                    // Cut off slightly more to ensure that we don't later have spurious
                    // same-original-drawing collisions when making a tail here.
                    const double fudgeFactor = 0.05;
                    t_stroke = core::mathUtility::lerp( *t_stroke, ess[ i ].t[ 1 ], fudgeFactor );

                    const auto newCutF = ess[ i ].f( *t_stroke );
                    cutF = std::min( 1., std::max( cutF, newCutF ) );
                }
            }
        }

        // Create temporary 'Stroke's representing the tailable regions of 'bs'.
        std::array< UniqueStroke, Endpoint::NumEndpoints > tr;
        std::array< bool, Endpoint::NumEndpoints > hasTail;
        for( size_t i = 0; i < Endpoint::NumEndpoints; i++ ) {
            hasTail[ i ] = ess_cutF[ i ] < 1.;
            if( hasTail[ i ] ) {
                tr[ i ] = ess[ i ].interval( ess_cutF[ i ], 1. ).asStroke();
            }
        }
        if( tr[ Start ] ) {
            // Let 'tr' be oriented along 'bs'
            tr[ Start ] = tr[ Start ]->reverse();
        }

        // No tails: easy case.
        if( !hasTail[ Start ] && !hasTail[ End ] ) {
            // Simplify the chain before converting to 'Stroke' in order to
            // reduce the amount of stitching.
            const auto cSimple = cComplex.simplified();
            storePretails = cSimple->stroke();
            storePreserve = BoundingInterval{ 0., 1. };
            return;
        }

        auto cTrimmed = cComplex.clone();
        if( singleSSChain ) {
            // Special case: we have two tails and both come out of the single 'Substroke' of 'cComplex'
            auto& ss = cTrimmed->substrokes.front();
            const auto fA = 1. - ess_cutF[ Start ];
            const auto fB = ess_cutF[ End ];
            if( fA >= fB ) {
                // The tailify cuts overlap in single-'Substroke' case. This is
                // the easy case when everything can become tails.
                storePretails = cComplex.stroke();
                return;
            }
            ss = ss.interval( fA, fB );
        } else {
            if( hasTail[ Start ] ) {
                auto& frontSS = cTrimmed->substrokes.front();
                frontSS = frontSS.interval( 1. - ess_cutF[ Start ], 1. );
            }
            if( hasTail[ End ] ) {
                auto& backSS = cTrimmed->substrokes.back();
                backSS = backSS.interval( 0., ess_cutF[ End ] );
            }
        }

        const auto cSimple = cTrimmed->simplified();
        if( cSimple->closed ) {
            THROW_UNEXPECTED;
        }
        // the part of 'bs' that must not be converted to tails.
        const auto preservePiece = cSimple->stroke();

        // To get the full 'storePretails', stitch 'preservePiece' with the non-null elements in
        // 'tr' and keep track of the stitch-T values to build an accurate 'storePreserve'.
        std::vector< double > stitchT;
        core::model::RawConstStrokes parts;
        if( tr[ Start ] && tr[ End ] ) {
            parts = { tr[ Start ].get(), preservePiece.get(), tr[ End ].get() };
            storePretails = core::model::stitchC0Strokes( parts, false, &stitchT );
            if( stitchT[ 0 ] < stitchT[ 1 ] ) {
                storePreserve = BoundingInterval{ stitchT[ 0 ], stitchT[ 1 ] };
            } // else assuming that 'preservePiece' is essentially length=0, so just
              // act is if there is no to-preserve for this one.
        } else if( tr[ Start ] ) {
            parts = { tr[ Start ].get(), preservePiece.get() };
            storePretails = core::model::stitchC0Strokes( parts, false, &stitchT );
            if( stitchT[ 0 ] < 1. ) {
                storePreserve = BoundingInterval{ stitchT[ 0 ], 1. };
            } // else just don't preserve anything
        } else if( tr[ End ] ) {
            parts = { preservePiece.get(), tr[ End ].get() };
            storePretails = core::model::stitchC0Strokes( parts, false, &stitchT );
            if( stitchT[ 0 ] > 0. ) {
                storePreserve = BoundingInterval{ 0, stitchT[ 0 ] };
            } // else just don't preserve anything
        } else {
            THROW_UNEXPECTED;
        }
    }

    void chainsToBlendStrokes( const chains::UniqueChains& chainsComplex )
    {
        const size_t numChains = chainsComplex.size();

        // Blend-strokes prior to having their ends converted to tails.
        UniqueStrokes pretails( numChains );
        // How much of each 'pretails' must not be converted to tails
        std::vector< PreserveInterval > preserve( numChains );

        const bool doProg = progBar && numChains;

        // Calculate 'pretails' and 'preserve'.
        if( doProg ) {
            progBar->startOnlyStage( "Pretails and preserve", static_cast< int >( numChains ) );
        }
        for( size_t i = 0; i < numChains; i++ ) {
            if( doProg ) {
                progBar->update( static_cast< int >( i ) );
            }
            const auto& chain = *chainsComplex[ i ];
            if( chain.substrokes.size() == 0 ) {
                THROW_UNEXPECTED;
            }
            pretailsAndPreserve( chain, pretails[ i ], preserve[ i ] );
            if( preserve[ i ] && preserve[ i ]->length() == 0. ) {
                THROW_RUNTIME( "Zero-length preserve interval not allowed; use boost::none instead" );
            }
        }

        // Make 'collProg'.
        if( doProg ) {
            progBar->startOnlyStage( "Make tails collider", static_cast< int >( numChains ) );
        }
        StrokeSegCollider collProg( collAB.bounds() );
        {
            // Put all of 'pretails' inside.
            for( size_t i = 0; i < numChains; i++ ) {
                if( doProg ) {
                    progBar->update( static_cast< int >( i ) );
                }
                const auto& fromPretails = pretails[ i ];
                const StrokePoly poly( *fromPretails, strokePolyLength( *fromPretails ) );
                collProg.addStroke( poly );
            }

            if( opts.preserveDrawing ) {
                const auto toPreserveID = *opts.preserveDrawing;
                for( const auto& pair : sToPoly ) {
                    const auto& stroke = pair.first;
                    if( drawings.whichDrawing( stroke ) == toPreserveID ) {
                        const auto& poly = pair.second;
                        collProg.addStroke( poly );
                    }
                }
            }
        }

        // Generate tails and actual final blend-strokes.
        UniqueStrokes blendStrokes;
        if( doProg ) {
            progBar->startOnlyStage( "Generate tails", static_cast< int >( numChains ) );
        }
        for( size_t i = 0; i < numChains; i++ ) {
            if( doProg ) {
                progBar->update( static_cast< int >( i ) );
            }

            auto& beforeTails = pretails[ i ];
            const auto& preserveInterval = preserve[ i ];

            // Easy case: not supposed to have any tails
            if( preserveInterval && preserveInterval == BoundingInterval{ 0., 1. } ) {
                blendStrokes.push_back( std::move( beforeTails ) );
                continue;
            }

            collProg.removeStroke( beforeTails.get() );

            tails::TailMaker tailMaker( *beforeTails, preserveInterval, collProg, *parent );
            auto withTails = tailMaker.result();
            const StrokePoly sPoly( *withTails, strokePolyLength( *withTails ) );
            collProg.addStroke( sPoly );
            blendStrokes.push_back( std::move( withTails ) );
        }

        if( doProg ) {
            progBar->startOnlyStage( "Cleanup" );
        }

        // The reason I'm doing this goofy after-the-fact fix is basically that the chain
        // code doesn't currently know how to represent a chain of 'Substroke's where i
        // is [a,0/1] of 's' and i+1 is [1/0,b] of 's'.
        //
        // After this call, 'blendStrokes' does not correspond with 'chainsComplex' anymore.
        blendStrokes = reconnectBrokenClosedStrokes( std::move( blendStrokes ), chainsComplex );

        // Add 'blendStrokes' to 'results'
        for( auto&& s : blendStrokes ) {
            results.push_back( std::move( s ) );
        }
    }

    /// Assuming 'blendStrokes' corresponds with 'chains', return updated 'blendStrokes' so that
    /// if two 'Stroke's in 'blendStrokes' have endpoints corresponding to T=0/1 ends of previously closed
    /// 'Stroke', then these two get stitched together.
    UniqueStrokes reconnectBrokenClosedStrokes( const UniqueStrokes& blendStrokes, const chains::UniqueChains& chains )
    {
        if( blendStrokes.size() != chains.size() ) {
            THROW_UNEXPECTED;
        }
        const size_t numChains = chains.size();

        // Find original-drawing closed strokes
        std::set< StrokeHandle > originalClosed;
        for( int i = 0; i < DrawingID::NumDrawings; i++ ) {
            drawings[ i ].forEach( [ & ]( const Stroke& s )
            {
                if( s.closed() ) {
                    originalClosed.emplace( &s );
                }
            } );
        }

        /// A reference to the start or end of some indexed thing.
        struct EndpointRef
        {
            /// index of a thing, e.g., of a 'Chain' in 'chains' or a 'Stroke' in 'blendStrokes'.
            size_t idx = 0;
            Endpoint endpoint = Endpoint::Start;
        };
        /// For when some two-ended thing needs to store an 'EndpointRef' at each of its ends.
        using EndpointRefs = std::array< boost::optional< EndpointRef >, Endpoint::NumEndpoints >;

        // Maps endpoints of items in 'originalClosed' to endpoints of 'Chain's in 'chains'.
        std::map< StrokeHandle, EndpointRefs > osToChains;
        for( size_t i = 0; i < numChains; i++ ) {
            const auto& c = *chains[ i ];
            if( c.closed ) {
                continue;
            }

            for( size_t j = 0; j < 2; j++ ) {
                const Endpoint ep = static_cast< Endpoint >( j );
                const bool start = ep == Endpoint::Start;
                if( c.hasTail( start ) ) {
                    continue;
                }
                const auto& endSS = start ? c.substrokes.front() : c.substrokes.back();
                const auto* const stroke = endSS.stroke;
                if( originalClosed.find( stroke ) == originalClosed.end() ) {
                    continue;
                }
                const auto tStroke = start ? endSS.t[ 0 ] : endSS.t[ 1 ];

                EndpointRef cRef;
                cRef.idx = i;
                cRef.endpoint = ep;

                if( tStroke == 0. ) {
                    osToChains[ stroke ][ 0 ] = cRef;
                } else if( tStroke == 1. ) {
                    osToChains[ stroke ][ 1 ] = cRef;
                }
            }
        }

        // Map the endpoints of an item in 'blendStrokes' to the endpoints of other items in 'blendStrokes'.
        std::map< StrokeHandle, EndpointRefs > bsToBS;
        for( const auto& pair : osToChains ) {
            const auto& refs = pair.second;
            if( !refs[ 0 ] || !refs[ 1 ] ) {
                continue;
            }
            if( refs[ 0 ]->idx == refs[ 1 ]->idx ) {
                continue;
            }
            bsToBS[  blendStrokes[ refs[ 0 ]->idx ].get()  ][  refs[ 0 ]->endpoint  ] = refs[ 1 ];
            bsToBS[  blendStrokes[ refs[ 1 ]->idx ].get()  ][  refs[ 1 ]->endpoint  ] = refs[ 0 ];
        }

        // Will not correspond with 'chains' like 'blendStrokes' used to.
        UniqueStrokes ret;

        // Use 'bsToBS' to build chains (forgive me) of blend-strokes that need to be stitched together.

        /// All 'Substroke's are either T=[0,1] or T=[1,0].
        using BlendStrokeChain = std::vector< Substroke >;
        std::set< StrokeHandle > usedBS;
        for( const auto& pair: bsToBS ) {
            const auto* const bs = pair.first;
            if( usedBS.find( bs ) != usedBS.end() ) {
                continue;
            }

            BlendStrokeChain chain{ Substroke( *bs, 0., 1. ) };
            bool closed = false;

            /// Return whether an element could be added. If the chain closes in this
            /// step, set 'closed' to true.
            const auto addToChain = [ & ]( bool addToFront ) -> bool
            {
                // endpoint-'Substroke' of 'chain'
                const auto& ess = addToFront ? chain.front() : chain.back();
                const auto* const bs = ess.stroke;
                const bool tInc = ess.tIncreasing();
                const auto it = bsToBS.find( bs );
                if( it == bsToBS.end() ) {
                    THROW_UNEXPECTED;
                } else {
                    const auto& bsEndpointRefs = it->second;
                    const Endpoint whichEndpoint = addToFront == tInc ? Endpoint::Start : Endpoint::End;
                    const auto& endpoint = bsEndpointRefs[ whichEndpoint ];
                    if( endpoint ) {
                        const auto* const nextBS = blendStrokes[ endpoint->idx ].get();

                        // Have we just discovered that 'chain' is closed?
                        const auto* const otherEndBS = ( addToFront ? chain.back() : chain.front() ).stroke;
                        if( nextBS == otherEndBS ) {
                            if( chain.size() == 1 ) {
                                THROW_UNEXPECTED;
                            }
                            closed = true;
                            return true;
                        }

                        const bool nextTInc = ( endpoint->endpoint == End ) == addToFront;
                        Substroke nextSS;
                        nextSS.stroke = nextBS;
                        nextSS.t = nextTInc ? std::array< double, 2 >{ 0., 1. } : std::array< double, 2 >{ 1., 0. };
                        if( addToFront ) {
                            chain.insert( chain.begin(), nextSS );
                        } else {
                            chain.push_back( nextSS );
                        }
                        return true;
                    } // else chain is done in this direction
                }
                return false;
            };

            while( true ) {
                const auto addedToFront = addToChain( true );
                if( closed ) {
                    break;
                }
                const auto addedToBack = addToChain( false );
                if( closed ) {
                    break;
                }
                if( !addedToFront && !addedToBack ) {
                    break;
                }
            }

            if( chain.size() == 1 ) {
                THROW_UNEXPECTED;
            }

            UniqueStrokes strokesToStitch;
            for( const auto& ss : chain ) {
                const auto* bs = ss.stroke;
                if( usedBS.find( bs ) == usedBS.end() ) {
                    usedBS.emplace( bs );
                } else {
                    THROW_UNEXPECTED;
                }
                strokesToStitch.push_back( ss.tIncreasing() ? bs->clone() : bs->reverse() );
            }

            // Sanity check: is this a bad stitch?
            if( !core::model::strokesAreApproxC0( strokesToStitch, closed, 1. ) ) {
                THROW_RUNTIME( "A bad stitch has been set up." );
            }

            auto stitched = core::model::stitchC0Strokes( strokesToStitch, closed );
            ret.push_back( std::move( stitched ) );
        }

        // Finally, put in 'ret' all the blend-strokes not involved with any chains.
        for( const auto& bs : blendStrokes ) {
            if( usedBS.find( bs.get() ) == usedBS.end() ) {
                ret.push_back( bs->clone() );
            }
        }

        return ret;
    }

    void perform()
    {                
        results.clear();

        if( progBar ) {
            progBar->startOnlyStage( "Finding topology" );
        }

        topology::FindTopology ft(
            drawings[ DrawingID::DrawingA ],
            drawings[ DrawingID::DrawingB ],
            sToPoly );

        topol = ft.topology();

        chains::ChainBuilder chainsBuilder( *topol, *parent, progBar );
        const auto chains = chainsBuilder.chains();

        // Special case where we preserve one of the drawings and make chains
        // purely from the other drawing.
        if( opts.preserveDrawing ) {
            const auto toPreserveID = *opts.preserveDrawing;

            // Put copy of 'toPreserve' in 'results'.
            const auto& toPreserve = drawings[ toPreserveID ];
            toPreserve.forEach(
            [ & ]( const Stroke& s )
            {
                results.push_back( s.clone() );
            } );

            // As a sanity check, make sure that nothing in 'chains' involves
            // 'Substroke's from 'toPreserve'.
            const auto otherD = otherDrawing( toPreserveID );
            for( const auto& c : chains ) {
                for( const auto& ss : c->substrokes ) {
                    if( drawings.whichDrawing( ss.stroke ) != otherD ) {
                        THROW_UNEXPECTED;
                    }
                }
            }
        }

        chainsToBlendStrokes( chains );
    }

    const BlendDrawings* const parent;
    const BlendOptions& opts;
    Drawings drawings;
    /// Stores only original-drawing (A/B) 'Stroke's.
    StrokeSegCollider collAB;
    /// Keep track of where same-'Drawing' 'Stroke's hit each other
    /// and themselves.
    DrawingToSameDrawingHits sameDrawingHits;
    /// Map from original-drawing 'Stroke' to data.
    std::map< StrokeHandle, StrokePoly > sToPoly;

    std::unique_ptr< topology::Topology > topol;
    std::vector< UniqueStroke > results;

    core::view::ProgressBar* progBar;
};

BlendDrawings::BlendDrawings( Drawings&& toOwn, const BlendOptions& opts, core::view::ProgressBar* progBar )
    : _imp( std::make_unique< Imp >( this, opts, std::move( toOwn ) ) )
{
    _imp->progBar = progBar;
}

BlendDrawings::~BlendDrawings()
{
}

void BlendDrawings::perform()
{
    _imp->perform();
}

const BlendOptions& BlendDrawings::options() const
{
    return _imp->opts;
}

const core::model::UniqueStrokes& BlendDrawings::result() const
{
    return _imp->results;
}

const Drawings& BlendDrawings::drawings() const
{
    return _imp->drawings;
}

const StrokeSegCollider& BlendDrawings::collAB() const
{
    return _imp->collAB;
}

size_t BlendDrawings::strokePolyLength( const Stroke& s ) const
{
    return _imp->strokePolyLength( s );
}

bool BlendDrawings::insideOriginalStroke( const core::model::Pos& p ) const
{
    for( const auto& pair : _imp->sToPoly ) {
        if( pair.second.contains( p ) ) {
            return true;
        }
    }
    return false;
}

const BlendDrawings::StrokeToPoly& BlendDrawings::originalStrokeToPoly() const
{
    return _imp->sToPoly;
}

const SameDrawingHits& BlendDrawings::sameDrawingHits( DrawingID dID ) const
{
    return _imp->sameDrawingHits[ dID ];
}

} // mashup
