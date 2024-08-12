#ifndef CORE_MATH_CANVASFIELD_H
#define CORE_MATH_CANVASFIELD_H

#include <Core/exceptions/runtimeerror.h>
#include <Core/model/posback.h>
#include <Core/model/lineback.h>

#include <Core/utility/boundingbox.h>
#include <Core/utility/mathutility.h>
#include <Core/utility/twodarray.h>

namespace core {
namespace math {

/// A field of 'T' defined over some bounding box 'bb' in canvas space as a grid,
/// where [0,0] in the field's 2D array corresponds to the top-left of 'bb'.
template< typename T >
class CanvasField : private core::TwoDArray< T >
{
public:
    using Base = typename core::TwoDArray< T >;
    using BoundingBox = core::BoundingBoxd;
    using Pos = model::Pos;

    enum GradientType
    {
        SimpleFill,
        Linear,
        Radial
    };

    struct Gradient
    {
        T val;
        GradientType type = SimpleFill;

        /// In canvas space. Not needed in 'SimpleFill' case
        Pos posA;
        Pos posB;

        /// [0,1]
        double alpha = 1.;
    };

    /// Create a field covering 'bounds' whose smaller dimension has
    /// 'minDimCells' ( > 0 ) cells.
    CanvasField( const BoundingBox& bounds, size_t minCellsRes, T initialVal )
        : _canvasBounds( bounds )
    {
        if( minCellsRes < 1 ) {
            THROW_UNEXPECTED;
        }
        const auto minDim = _canvasBounds.minDim();
        _cellWidth = minDim / static_cast< double >( minCellsRes );

        const auto maxDim = _canvasBounds.maxDim();
        const auto maxCellsRes = static_cast< size_t >( std::ceil( maxDim / _cellWidth ) );

        if( _canvasBounds.widthExclusive() > _canvasBounds.heightExclusive() ) {
            Base::recreate( static_cast< int >( maxCellsRes ), static_cast< int >( minCellsRes ), initialVal );
        } else {
            Base::recreate( static_cast< int >( minCellsRes ), static_cast< int >( maxCellsRes ), initialVal );
        }
    }

    const Base& asScalarArray() const
    {
        return *this;
    }

    void applyGradient( const Gradient& g )
    {
        switch( g.type ) {
        case SimpleFill: {
            Base::forEveryPos(
                [ &g ]( T& valRef )
                {
                    valRef = valRef + g.alpha * ( g.val - valRef );
                } );
            break;
        }
        case Radial: {
            AlphaMask mask;
            radialGradientAlphaMask( g.posA, g.posB, g.alpha, mask );
            applyValUsingAlphaMask( g.val, mask );
            break;
        }
        case Linear: {
            AlphaMask mask;
            linearGradientAlphaMask( g.posA, g.posB, g.alpha, mask );
            applyValUsingAlphaMask( g.val, mask );
            break;
        }
        default: {
            THROW_UNEXPECTED;
        }
        }
    }

    T interp( const Pos& canvas ) const
    {
        const auto grid = gridSpace( canvas );
        return Base::interpolate( grid );
    }

private:
    /// Has same dimensions as 'this'.
    using AlphaMask = core::TwoDArray< double >;

    /// 'a' and 'b' in canvas space; 'aAlpha' in [0,1]
    void radialGradientAlphaMask( const Pos& a, const Pos& b, double aAlpha, AlphaMask& store ) const
    {
        const auto aGrid = gridSpace( a );
        const auto bGrid = gridSpace( b );
        const auto radGrid = ( bGrid - aGrid ).length();
        store.recreate( Base::width(), Base::height(), 0. );
        if( core::mathUtility::closeEnoughToZero( radGrid ) ) {
            return;
        }
        const model::Seg gridSeg( aGrid, bGrid );
        store.set(
            [ & ]( int x, int y ) {
                const Pos gridPos( static_cast< double >( x ), static_cast< double >( y ) );
                return aAlpha * ( 1. - std::min< double >( 1., ( gridPos - aGrid ).length() / radGrid ) );
            } );
    }

    /// 'a' and 'b' in canvas space; 'aAlpha' in [0,1]
    void linearGradientAlphaMask( const Pos& a, const Pos& b, double aAlpha, AlphaMask& store ) const
    {
        const auto aGrid = gridSpace( a );
        const auto bGrid = gridSpace( b );
        const model::Seg gridSeg( aGrid, bGrid );
        store.recreate( Base::width(), Base::height(), 0. );
        store.set(
        [ & ]( int x, int y ) {
            double unused = 0.;
            const auto closest = core::mathUtility::closestPointOnLineSegment(
                core::IntCoord( x, y ).toVector2(), aGrid, bGrid, unused );
            return ( 1. - gridSeg.t( closest ) ) * aAlpha;
        } );
    }

    void applyValUsingAlphaMask( T val, const AlphaMask& alphaMask )
    {
        if( alphaMask.size() != Base::size() ) {
            THROW_UNEXPECTED;
        }
        const auto w = Base::width();
        const auto h = Base::height();
        for( int x = 0; x < w; x++ ) {
            for( int y = 0; y < h; y++ ) {
                auto& toUpdate = Base::getRef( x, y );
                const auto& alpha = alphaMask.getRef( x, y );
                toUpdate = toUpdate + alpha * ( val - toUpdate );
            }
        }
    }

    /// Convert from canvas space to {[0,cellsWide],[0,cellsHigh]} (without clamping).
    Pos gridSpace( const Pos& canvas ) const
    {
        const auto fromTL = canvas - _canvasBounds.topLeft();
        return fromTL / _cellWidth;
    }

    double _cellWidth = 0.;
    const BoundingBox _canvasBounds;
};

} // math
} // core

#endif // #include
