#include <topology/findtopology.h>

#include <drawingid.h>
#include <strokeback.h>
#include <strokepoly.h>
#include <topology/strokeintersection.h>
#include <topology/strokeintervals.h>
#include <topology/topology.h>

#include <Core/model/polyline.h>
#include <Core/model/lineback.h>

#include <Core/utility/mathutility.h>

#include <algorithm>
#include <map>
#include <set>

namespace mashup {
namespace topology {

using Pos = core::model::Pos;
using Normal = Pos;
using Seg = core::model::Seg;

struct FindTopology::Imp
{
    Imp( const Drawing& a, const Drawing& b, const StrokeToPoly& sToPoly )
        : drawings{ &a, &b }
    {
        const auto ingest = [ & ]( DrawingID dId, const Drawing& d )
        {
            polys[ dId ].resize( d.numStrokes() );
            size_t i = 0;
            d.forEach( [ & ]( const Stroke& s )
            {
                polys[ dId ][ i++ ] = &( sToPoly.find( &s )->second );
            } );
        };
        
        ingest( DrawingID::DrawingA, a );
        ingest( DrawingID::DrawingB, b );
    }

    /// Return whether 'p' is inside one or more of the 'Stroke's in the indicated drawing.
    bool isInsideSomething( const Pos& p, DrawingID dId, const std::vector< size_t >& indicesToCheck ) const
    {
        const auto& strokePolys = polys[ dId ];
        for( const auto idx : indicesToCheck ) {
            const auto* const poly = strokePolys[ idx ];
            if( poly->contains( p ) ) {
                return true;
            }
        }
        return false;
    }

    /// Return whether 'seg' crosses any of the stroke-polygons identified by 'dId' and 'indicesToCheck'.
    bool crossesSomething( const Seg& seg, DrawingID dId, const std::vector< size_t >& indicesToCheck ) const
    {
        const auto& strokePolys = polys[ dId ];
        for( const auto idx : indicesToCheck ) {
            const auto* const poly = strokePolys[ idx ];
            if( poly->outlineCrosses( seg ) ) {
                return true;
            }
        }
        return false;
    }

    /// Return values in [0,1] where the 'side' side of 'sPoly' collides with other-drawing
    /// stroke-polys indicated by 'otherPolysIndices' (and 'otherDrawingId').
    std::set< double > strokeSideCritT(
                                  const StrokePoly& sPoly,
                                  StrokeSide side,
        DrawingID otherDrawingId,
                                  const std::vector< size_t >& otherPolysIndices )
    {
        const auto& otherPolys = polys[ otherDrawingId ];

        std::set< double > ret;

        for( size_t i = 0; i < sPoly.pointsPerSide() - 1; i++ ) {

            const auto tA = sPoly.t[ i ];
            const auto tB = sPoly.t[ i + 1 ];

            const auto& pA = sPoly.sides[ side ][ i ];
            const auto& pB = sPoly.sides[ side ][ i + 1 ];
            const Seg seg{ pA, pB };

            for( const auto otherS : otherPolysIndices ) {
                const auto* const otherPoly = otherPolys[ otherS ];
                otherPoly->forEachSeg(
                            [ & ]( const Seg& otherStrokeSeg,
                                   const Normal&,
                                   double tA_otherStroke,
                                   double tB_otherStroke )
                {
                    Pos hitPos;
                    if( core::mathUtility::segmentsIntersect( seg, otherStrokeSeg, hitPos ) ) {

                        const auto t_myStroke = core::mathUtility::lerp( tA, tB, seg.t( hitPos ) );
                        ret.emplace( t_myStroke );

                        // Record for later use
                        {
                            const auto t_otherStroke = core::mathUtility::lerp(
                                        tA_otherStroke,
                                        tB_otherStroke,
                                        otherStrokeSeg.t( hitPos ) );

                            StrokeIntersection si;
                            si.stroke[ 0 ] = sPoly.stroke;
                            si.stroke[ 1 ] = otherPoly->stroke;
                            si.t[ 0 ] = t_myStroke;
                            si.t[ 1 ] = t_otherStroke;
                            intersections.push_back( si );
                        }
                    }
                } );
            }
        }

        return ret;
    }
    
    void processStroke( size_t sIndex, DrawingID dId, Topology& addTo )
    {
        const auto* const stroke = drawings[ dId ]->stroke( sIndex );
        const auto& sPoly = *polys[ dId ][ sIndex ];
        if( !sPoly.participates() ) {
            return;
        }

        const auto otherDId = otherDrawing( dId );
        const auto& otherPolys = polys[ otherDId ];

        // Determine which of 'otherPolys' we need to test against 'stroke' (partly for optimization, partly to ignore non-participating strokes).
        std::vector< size_t > otherPolysIndices;
        {
            const auto& sBounds = sPoly.bounds;
            for( size_t i = 0; i < otherPolys.size(); i++ ) {
                if( otherPolys[ i ]->participates() ) {
                    if( sBounds.intersects( otherPolys[ i ]->bounds ) ) {
                        otherPolysIndices.push_back( i );
                    }
                }
            }
        }

        // Find "critical T" values: where the sides of 'sPoly' intersect polys from 'otherPolysIndices'.
        auto combinedT = strokeSideCritT( sPoly, Left, otherDId, otherPolysIndices );
        const auto rightT = strokeSideCritT( sPoly, Right, otherDId, otherPolysIndices );
        for( auto& t : rightT ) {
            combinedT.emplace( t );
        }
        combinedT.emplace( 0. );
        combinedT.emplace( 1. );

        TIntervals unoccluded; // for 'stroke'
        {
            boost::optional< TInterval > prog;
            auto itA = combinedT.begin();
            while( true ) {
                auto itB = itA;
                itB++;

                if( itB == combinedT.end() ) {
                    break;
                }

                const auto tA = *itA;
                const auto tB = *itB;
                const auto tMid = ( tA + tB ) / 2.;

                const Seg perpSeg(
                            sPoly.onSide( tMid, Left ),
                            sPoly.onSide( tMid, Right ) );
                const auto perpMid = perpSeg.midpoint();
                if( crossesSomething( perpSeg, otherDId, otherPolysIndices )
                 || isInsideSomething( perpMid, otherDId, otherPolysIndices ) ) {
                    // [tA,tB] is occluded; do not add to 'prog'
                    if( prog.is_initialized() ) {
                        unoccluded.push_back( *prog );
                        prog = boost::none;
                    }
                } else {
                    if( prog.is_initialized() ) {
                        prog = TInterval{ prog->min(), tB };
                    } else {
                        prog = TInterval{ tA, tB };
                    }
                }

                itA++;
            }
            if( prog.is_initialized() ) {
                unoccluded.push_back( *prog );
            }
        }

        if( unoccluded.size() ) {
            const StrokeIntervals intervals( unoccluded );
            addTo.addStroke( *stroke, intervals );
        }
    }

    std::unique_ptr< Topology > topology()
    {
        intersections.clear();

        auto ret = std::make_unique< Topology >();
        for( size_t i = 0; i < DrawingID::NumDrawings; i++ ) {
            const auto& drawing = drawings[ i ];
            for( size_t j = 0; j < drawing->numStrokes(); j++ ) {
                processStroke( j, static_cast< DrawingID >( i ), *ret );
            }
        }

        // Now that strokes are added to 'ret', get the crossings
        // updated by processing the 'intersections' we've collected on the way.
        for( const auto& i : intersections ) {
            ret->addStrokeIntersection( i );
        }

        ret->doneAdding();
        return ret;
    }

    // Each is the same size as the number of strokes in the corresponding 'Drawing'.
    std::array< StrokePolyHandles, DrawingID::NumDrawings > polys;
    const std::array< const Drawing*, DrawingID::NumDrawings > drawings;
    std::vector< StrokeIntersection > intersections;
};

FindTopology::FindTopology( const Drawing& a, const Drawing& b, const StrokeToPoly& sToPoly )
    : _imp( std::make_unique< Imp >( a, b, sToPoly ) )
{
}

FindTopology::~FindTopology()
{
}

std::unique_ptr< Topology > FindTopology::topology()
{
    auto ret = _imp->topology();    
    return ret;
}

} // topology
} // mashup
