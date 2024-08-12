#ifndef CORE_MATH_SEGCOLLIDERGRID_H
#define CORE_MATH_SEGCOLLIDERGRID_H

#include <Core/model/boundingboxback.h>
#include <Core/model/lineback.h>
#include <Core/model/posback.h>

#include <Core/utility/intcoord.h>
#include <Core/utility/mathutility.h>
#include <Core/utility/twodarray.h>

#include <boost/optional.hpp>

#include <set>

namespace core {
namespace math {

/// Grid structure representing a rectangle of space in which line segments are stored, each with
/// a 'Metadata', the purpose being to accelerate intersection detection.
template< typename Metadata >
class SegColliderGrid
{
public:
    using Pos = model::Pos;
    using IPos = core::IntCoord;
    using SetOfIPos = std::set< IPos >;
    using Seg = model::Seg;

    struct SegWithData
    {
        Seg seg;
        Metadata metadata;
    };
    using SegsWithData = std::vector< SegWithData >;
    using CellContents = SegsWithData;

    /// A yes/no question re. the metadata associated with a stored line segment.
    using MetadataPredicate = std::function< bool( const Metadata& ) >;
    /// A yes/no question re. a 'Seg' and its associated 'Metadata'.
    using SWDPredicate = std::function< bool( const SegWithData& ) >;

    SegColliderGrid( const model::BoundingBox& canvasBounds, int minCellsDim )
    {
        _canvasRect = canvasBounds;
        const auto minDimCanvas = canvasBounds.minDim();
        _cellWidth = minDimCanvas / static_cast< double >( minCellsDim );
        const int maxCellsDim = static_cast< int >( std::ceil( canvasBounds.maxDim() / _cellWidth ) );

        if( canvasBounds.widthExclusive() > canvasBounds.heightExclusive() ) {
            _grid.recreate( maxCellsDim, minCellsDim );
        } else {
            _grid.recreate( minCellsDim, maxCellsDim );
        }
    }

    const model::BoundingBox& bounds() const
    {
        return _canvasRect;
    }

    IPos cellCoords( const Pos& canvas ) const
    {
        const auto p = arrayPos( canvas );
        return { int( p.x() ), int( p.y() ) };
    }

    void addSeg( const Seg& seg, const Metadata& data, SetOfIPos* storeInvolvedCoords = nullptr )
    {
        SegWithData swd;
        swd.seg = seg;
        swd.metadata = data;

        const auto coords = checkCoords( seg );
        for( const auto& coord : coords ) {

            const auto x = coord.x();
            const auto y = coord.y();

            // Dilate the new info 3x3
            for( int x2 = x - 1; x2 <= x + 1; x2++ ) {
                for( int y2 = y - 1; y2 <= y + 1; y2++ ) {
                    const IPos coord( x2, y2 );
                    if( _grid.isValidCoord( coord ) ) {
                        auto& cell = _grid.getRef( coord );
                        cell.push_back( swd );
                        if( storeInvolvedCoords ) {
                            storeInvolvedCoords->emplace( coord );
                        }
                    }
                }
            }
        }
    }

    /// Return whether the line segment 'a'->'b' hits any stored line segment
    /// that passes 'include' (ignore 'include' if it is null).
    bool hitsAny( const Pos& a, const Pos& b, SWDPredicate include ) const
    {
        const Seg aToB{ a, b };
        const auto coordsToCheck = checkCoords( aToB );
        for( const auto& coord : coordsToCheck ) {
            if( _grid.isValidCoord( coord ) ) {
                const auto& bin = _grid.getRef( coord );
                for( const auto& swd : bin ) {
                    if( include && !include( swd ) ) {
                        continue;
                    }
                    const auto& seg = swd.seg;
                    Pos hit;
                    if( core::mathUtility::segmentsIntersect( aToB, seg, hit ) ) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void clear()
    {
        _grid.forEveryPos(
        []( CellContents& bin )
        {
            bin.clear();
        } );
    }

    size_t numSegs() const
    {
        size_t ret = 0;
        _grid.forEveryPos(
        [ &ret ]( const CellContents& bin )
        {
            ret += bin.size();
        } );
        return ret;
    }

    bool empty() const
    {
        return numSegs() == 0;
    }

    /// Remove all stored segments whose metadata satisfies 'removeIfTrue'.
    void removeSegs( MetadataPredicate removeIfTrue )
    {
        _grid.forEveryPos(
        [ &removeIfTrue ]( CellContents& bin )
        {
            CellContents filtered;
            for( const auto& pair : bin ) {
                if( !removeIfTrue( pair.metadata ) ) {
                    filtered.push_back( pair );
                }
            }
            bin = filtered;
        } );
    }

    /// Return the distance from 'posCanvas' to the nearest segment satisfying 'segsToConsider' (which can be nullptr) that is closer
    /// than 'maxDistAllowed' (whose purpose is to reduce computation time as much as possible). Return boost::none if no match found.
    boost::optional< double > distToNearestSeg( const Pos& posCanvas, MetadataPredicate segsToConsider, double maxDistAllowed ) const
    {
        int centerX = 0;
        int centerY = 0;
        {
            const auto topLeft = _canvasRect.topLeft();
            const Pos pos = posCanvas - topLeft;
            centerX = static_cast< int >( pos.x() / _cellWidth );
            centerY = static_cast< int >( pos.y() / _cellWidth );
        }

        // An integral (number-of-cells) width guaranteed to include all marked points within 'maxDistThatMatters' of 'pos'.
        const auto maxNeighborhoodWidth = neighborhoodWidth( maxDistAllowed );

        // Move through neighborhoods 1X1, 3X3, 5X5... up to 'maxNeighborhoodWidth', looking for any marked positions.
        boost::optional< double > closestDist;
        for( size_t neighborhoodWidth = 1; neighborhoodWidth <= maxNeighborhoodWidth; neighborhoodWidth += 2 ) {

            const int halfMinOne = static_cast< int >( neighborhoodWidth ) / 2;
            const int left = centerX - halfMinOne;
            const int top = centerY - halfMinOne;
            const int right = centerX + halfMinOne;
            const int bottom = centerY + halfMinOne;

            // Walk around the outside of the neighborhood.
            const int numPerimeterCells = std::max< int >( static_cast< int >( neighborhoodWidth ) * 4 - 4, 1 );
            for( int i = 0; i < numPerimeterCells; i++ ) {
                int x, y;
                if( i < neighborhoodWidth ) {
                    x = left + i;
                    y = top;
                } else if( i < 2 * neighborhoodWidth ) {
                    x = left + ( i - static_cast< int >( neighborhoodWidth ) );
                    y = bottom;
                } else if ( i < 3 * neighborhoodWidth - 2 ) {
                    x = left;
                    y = top + i + 1 - 2 * static_cast< int >( neighborhoodWidth );
                } else {
                    x = right;
                    y = top + i + 1 - ( 3 * static_cast< int >( neighborhoodWidth ) - 2 );
                }

                if( !_grid.isValidCoord( x, y ) ) {
                    continue;
                }
                const auto& bin = _grid.getRef( x, y );
                for( const auto& pair : bin ) {
                    if( !segsToConsider || segsToConsider( pair.metadata ) ) {
                        // We've found a segment.
                        const auto& seg = pair.seg;
                        double storeDist = 0.;
                        core::mathUtility::closestPointOnLineSegment( posCanvas, seg.a, seg.b, storeDist );
                        if( !closestDist.is_initialized() || storeDist < closestDist ) {
                            closestDist = storeDist;
                        }
                    }
                }
            }
        }
        return closestDist;
    }

protected:
    /// Return the size (an odd number in cells) of a square neighborhood s.t. if there is a point 'p'
    /// anywhere in the neighborhood's center cell, then there is no point 'q' within 'range' or 'p'
    /// that does not fall in the neighborhood.
    size_t neighborhoodWidth( double range ) const
    {
        // If our neighborhood center were at the corner of the box, the neighborhood would still
        // contain the whole grid.
        const size_t maxDimVal = std::max< size_t >( _grid.width(), _grid.height() ) * 2 + 1; // Must be odd.
        return std::min( maxDimVal,
                         static_cast< size_t >( std::ceil( range / _cellWidth ) ) * 2 + 1 );

    }

    /// Return 'canvasPos' in ([0,cellsWide],[0,cellsHigh])
    Pos arrayPos( const Pos& canvasPos ) const
    {
        return Pos {
            ( canvasPos.x() - _canvasRect.topLeft().x() ) / _cellWidth,
            ( canvasPos.y() - _canvasRect.topLeft().y() ) / _cellWidth
        };
    }

    std::vector< IPos > checkCoords( const Seg& seg ) const
    {
        const auto start = arrayPos( seg.a );
        const auto end = arrayPos( seg.b );
        return core::rasterizeSegment_floatingPoint( start, end );
    }

    core::TwoDArray< CellContents > _grid;
    double _cellWidth;
    model::BoundingBox _canvasRect;
};

} // math
} // core

#endif // #include
