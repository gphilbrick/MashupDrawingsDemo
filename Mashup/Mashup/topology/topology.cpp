#include <topology/topology.h>

#include <drawings.h>
#include <substroke.h>
#include <topology/crossing.h>
#include <topology/strokeintersection.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/model/curveback.h>

#include <list>
#include <set>

namespace mashup {
namespace topology {

namespace {

using Pos = core::model::Pos;

/// Ordinally identifies some occluded interval on some 'Stroke'.
using OccludedStrokeInterval = std::pair< StrokeHandle, size_t >;

/// Combines a 'Crossing' (needed by external code) with 'occludedIntervals' (used
/// to build topology in progress).
struct Xing
{
    std::set< OccludedStrokeInterval > occludedIntervals;
    /// Usable by outside code.
    std::unique_ptr< Crossing > crossing;
};

// iterators are persistent with push_back
using Xings = std::list< Xing >;
using XingsIt = Xings::iterator;

/// Data for 'Stroke' 's'
struct StrokeData
{
    /// Return the index of the occluded interval closest to 't', or 0 if there are
    /// no occluded intervals.
    size_t closestOccludedInterval( double t ) const
    {
        size_t bestOccluded = 0;
        double bestDist = std::numeric_limits< double >::max();
        for( size_t i = 0; i < occluded.size(); i++ ) {
            const auto& interval = occluded[ i ];
            double distTo = 0.;
            if( !interval.contains( t ) ) {
                distTo = std::min< double >(
                            std::abs( t - interval.min() ),
                            std::abs( t - interval.max() ) );
            }
            if( distTo < bestDist ) {
                bestDist = distTo;
                bestOccluded = i;
            }
        }
        return bestOccluded;
    }

    /// Given 'occIdx', which corresponds to one of the 'Crossing's that 's' goes through,
    /// i.e., to one of the 'occluded' intervals, return 0, 1, or 2 'Substrokes' from
    /// 's' that feed into that 'Crossing'.
    std::vector< Substroke > substrokesPointingIntoOccluded( size_t occIdx ) const
    {
        boost::optional< size_t > unoccBefore, unoccAfter;
        intervals.unoccludedAdjacentTo( occIdx, unoccBefore, unoccAfter );

        std::vector< Substroke > ret;
        if( unoccBefore ) {
            ret.push_back( substrokes[ *unoccBefore ] );
        }
        if( unoccAfter ) {
            ret.push_back( substrokes[ *unoccAfter ].reverse() );
        }
        return ret;
    }

    /// Assuming that 'ss' is or lies inside one of the unoccluded intervals on 's',
    /// return a handle to the 'Router' associated with the end of 'ss' (or nullptr if
    /// 'ss' does not end at a crossing).
    Crossing* findCrossing( const Substroke& ss ) const
    {
        const auto occIdx = intervals.crossingIndex( ss );
        if( occIdx.is_initialized() ) {
            return xings[ *occIdx ]->crossing.get();
        } else {
            return nullptr;
        }
    }

    /// 'occIdx' identifies one of the occluded intervals.
    /// Return this interval expanded to include the unoccluded intervals
    /// on either side, if there.
    TInterval envelopeAroundOccluded( size_t occIdx ) const
    {
        if( occIdx >= occluded.size() ) {
            THROW_UNEXPECTED;
        }

        boost::optional< size_t > unoccBefore, unoccAfter;
        intervals.unoccludedAdjacentTo( occIdx, unoccBefore, unoccAfter );
        const auto& occ = occluded[ occIdx ];
        return TInterval
        {
            unoccBefore.is_initialized() ? unoccluded[ *unoccBefore ].min() : occ.min(),
            unoccAfter.is_initialized() ?  unoccluded[ *unoccAfter ].max() : occ.max()
        };
    }

    StrokeIntervals intervals;

    /// Sorted by increasing T.
    TIntervals occluded;
    /// Corresponds with 'occluded'.
    std::vector< XingsIt > xings;

    /// Must have size > 0, sorted by increasing T.
    TIntervals unoccluded;
    /// Corresponds with 'unoccluded', each T-increasing.
    std::vector< Substroke > substrokes;
};

} // unnamed

struct Topology::Imp
{
    void addStroke( const Stroke& s, const StrokeIntervals& intervals )
    {
        if( !intervals.anyUnoccluded() ) {
            // 's' is completely occluded and should be ignored.
            return;
        }

        if( strokeData.find( &s ) != strokeData.end() ) {
            THROW_RUNTIME( "Tried to addStroke redundantly" );
        }

        auto& sData = strokeData[ &s ];
        sData.intervals = intervals;
        sData.occluded = sData.intervals.intervals( true );
        sData.unoccluded = sData.intervals.intervals( false );

        // For every occluded interval, create a 'Crossing'
        for( size_t i = 0; i < sData.occluded.size(); i++ ) {
            xings.push_back( {} );
            auto& crossing = xings.back();
            crossing.occludedIntervals.emplace( OccludedStrokeInterval{ &s, i } );
            sData.xings.push_back( std::next( xings.end(), -1 ) );
        }

        // For every unoccluded interval, create a 'Substroke'.
        for( size_t i = 0; i < sData.unoccluded.size(); i++ ) {
            const auto& interval = sData.unoccluded[ i ];
            sData.substrokes.push_back( Substroke{ s, interval.min(), interval.max() } );
        }
    }

    void addStrokeIntersection( const StrokeIntersection& i )
    {
        const auto tA = i.t[ 0 ];
        const auto tB = i.t[ 1 ];

        auto aIt = strokeData.find( i.stroke[ 0 ] );
        auto bIt = strokeData.find( i.stroke[ 1 ] );
        if( aIt == strokeData.end() || bIt == strokeData.end() ) {
            return;
        }

        auto& aData = aIt->second;
        auto& bData = bIt->second;

        // It only makes sense to proceed if both 'Stroke's even _have_ occluded intervals.
        if( aData.occluded.size() == 0 || bData.occluded.size() == 0 ) {
            // This is a suspicious situation: why is external code telling us about an
            // intersection between these two strokes then?
            return;
        }

        const auto& aCross = aData.xings[ aData.closestOccludedInterval( tA ) ];
        auto bCross = bData.xings[ bData.closestOccludedInterval( tB ) ];

        if( aCross != bCross ) {
            // Merge the two 'Crossings'. The one pointed at by 'aCross' becomes the merger,
            // while the one pointed at by 'bCross' gets deleted.
            for( const auto& p : bCross->occludedIntervals ) {
                aCross->occludedIntervals.emplace( p );
            }

            // Any 'StrokeData' holding 'bCross' should hold 'aCross' instead.
            for( auto& pair : strokeData ){
                auto& data = pair.second;
                for( auto& it : data.xings ) {
                    if( it == bCross ) {
                        it = aCross;
                    }
                }
            }
            xings.erase( bCross );
        } // else these two already refer to the same 'Crossing'.
    }

    /// Return 'Substroke's representing all the unoccluded intervals of all participating
    /// 'Stroke's, in no particular order.
    std::vector< Substroke > unoccludedSubstrokes( const Drawings& drawings ) const
    {
        std::vector< Substroke > ret;

        size_t numKeysUsed = 0;
        for( int dIdx = 0; dIdx < DrawingID::NumDrawings; dIdx++ ) {
            drawings[ dIdx ].forEach(
            [ & ]( const Stroke& s ) {
                const auto it = strokeData.find( &s );
                if( it != strokeData.end() ) {
                    numKeysUsed++;
                    const auto& sData = it->second;
                    for( const auto& ss : sData.substrokes ) {
                        ret.push_back( ss );
                    }
                }
            } );
        }

        if( numKeysUsed != strokeData.size() ) {
            THROW_RUNTIME( "Topology has StrokeHandle keys not found in original 'Drawings'." );
        }

        return ret;
    }

    /// For every 'Xing', build its contained 'Crossing'.
    /// This should happen only once.
    void buildCrossings()
    {
        for( auto& xing : xings ) {
            xing.crossing = std::make_unique< Crossing >();

            // Come up with all the 'Substroke's feeding into 'xing'.
            for( const auto& occ : xing.occludedIntervals ) {
                const auto occIdx = occ.second;
                const auto sDataIt = strokeData.find( occ.first );
                if( sDataIt == strokeData.end() ) {
                    THROW_UNEXPECTED;
                }

                const auto& sData = sDataIt->second;
                const auto substrokes = sData.substrokesPointingIntoOccluded( occIdx );
                if( substrokes.size() == 2 ) {
                    xing.crossing->add( substrokes[ 0 ], substrokes[ 1 ] );
                } else if( substrokes.size() == 1 ) {
                    xing.crossing->add( substrokes[ 0 ] );
                }

                const auto envelopeInterval = sData.envelopeAroundOccluded( occIdx );
                Substroke envelopeSS( *occ.first, envelopeInterval.min(), envelopeInterval.max() );
                xing.crossing->addEnvelopeAroundOccluded( envelopeSS );
            }            
        }
    }

    Crossing* findCrossing( const Substroke& ss ) const
    {
        auto it = strokeData.find( ss.stroke );
        return it->second.findCrossing( ss );
    }

    bool originallyConnected( const Substroke& a, const Substroke& b ) const
    {
        const auto* const crossing = findCrossing( a );
        if( crossing ) {
            // 'crossing' expects 'a' and 'b' to point at each other, whereas our 'a' and 'b'
            // are currently "consistently" oriented.
            return crossing->originallyConnected( a, b.reverse() );
        } else {
            return false;
        }
    }

    /// For each 'Stroke' (of either drawing), what are the occluded and unoccluded intervals?
    std::map< StrokeHandle, StrokeData > strokeData;
    Xings xings;
};

Topology::Topology() : _imp( std::make_unique< Imp >() )
{
}

Topology::~Topology()
{
}

void Topology::addStroke( const Stroke& s, const StrokeIntervals& intervals )
{
    _imp->addStroke( s, intervals );
}

void Topology::addStrokeIntersection( const StrokeIntersection& i )
{
    _imp->addStrokeIntersection( i );
}

bool Topology::originallyConnected( const Substroke& a, const Substroke& b ) const
{
    return _imp->originallyConnected( a, b );
}

std::vector< Substroke > Topology::unoccludedSubstrokes( const Drawings& d ) const
{
    return _imp->unoccludedSubstrokes( d );
}

void Topology::doneAdding()
{
    _imp->buildCrossings();
}

std::vector< const Crossing* > Topology::crossings() const
{
    std::vector< const Crossing* > ret;
    for( const auto& xing : _imp->xings ) {
        const auto* const crossing = xing.crossing.get();
        if( !crossing ) {
            // Make sure buildCrossings was called.
            THROW_UNEXPECTED;
        } else {
            ret.push_back( crossing );
        }
    }
    return ret;
}

const Crossing* Topology::findCrossing( const Substroke& ss ) const
{
    return _imp->findCrossing( ss );
}

} // topology
} // mashup
