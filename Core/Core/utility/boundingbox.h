#ifndef CORE_BOUNDINGBOX_H
#define CORE_BOUNDINGBOX_H

#include <Core/utility/vector2.h>
#include <Core/utility/intcoord.h>
#include <Core/utility/boundinginterval.h>

#include <boost/optional.hpp>

#include <algorithm>
#include <vector>

namespace core {

template< typename T, typename TCoord >
class BoundingBox
{
public:
    /// Create an instance which cannot be used until at least one point has been added.
    explicit BoundingBox()
    {
        _xMin = std::numeric_limits< T >::max();
        _xMax = std::numeric_limits< T >::lowest();
        _yMin = std::numeric_limits< T >::max();
        _yMax = std::numeric_limits< T >::lowest();
    }

    BoundingBox( const TCoord* array, size_t numPoints )
    {
        if( numPoints == 0 )
        {
            throw "illegal";
        }
        TCoord first = array[ 0 ];
        _xMin=first.x();
        _xMax=first.x();
        _yMin=first.y();
        _yMax=first.y();

        for( size_t i = 1; i < numPoints; i++ )
        {
            addPoint( array[ i ] );
        }
    }

    BoundingBox( const TCoord& a, const TCoord& b )
    {
        _xMin=a.x();
        _xMax=a.x();
        _yMin=a.y();
        _yMax=a.y();
        addPoint(b);
    }

    BoundingBox( const BoundingInterval<T>& xInterval, const BoundingInterval<T>& yInterval )
    {
        _xMin=xInterval.min();
        _xMax=xInterval.max();
        _yMin=yInterval.min();
        _yMax=yInterval.max();
    }

    BoundingBox(T xMin, T yMin, T xMax, T yMax)
    {
        _xMin=xMin;
        _xMax=xMax;
        _yMin=yMin;
        _yMax=yMax;
    }

    BoundingBox( const std::vector<TCoord>& points )
    {
        TCoord first = points[0];
        _xMin=first.x();
        _xMax=first.x();
        _yMin=first.y();
        _yMax=first.y();

        for(size_t i=1; i<points.size(); i++)
        {
            addPoint(points[i]);
        }
    }

    bool operator == ( const BoundingBox& other ) const
    {
        return _xMin == other._xMin
            && _xMax == other._xMax
            && _yMin == other._yMin
            && _yMax == other._yMax;
    }

    /// 'idx' must be in [0,3]
    void sideSegment( int idx, TCoord& storeA, TCoord& storeB ) const
    {
        switch( idx ) {
        case 0: {
            storeA = topLeft();
            storeB = topRight();
            break;
        }
        case 1: {
            storeA = topRight();
            storeB = bottomRight();
            break;
        }
        case 2: {
            storeA = bottomRight();
            storeB = bottomLeft();
            break;
        }
        case 3: {
            storeA = bottomLeft();
            storeB = topLeft();
            break;
        }
        }
    }

    TCoord topLeft() const
    {
        return TCoord(_xMin,_yMin);
    }

    TCoord topRight() const
    {
        return TCoord(_xMax,_yMin);
    }

    TCoord bottomLeft() const
    {
        return TCoord(_xMin,_yMax);
    }

    TCoord bottomRight() const
    {
        return TCoord(_xMax,_yMax);
    }

    double xMax() const
    {
        return _xMax;
    }

    double xMin() const
    {
        return _xMin;
    }

    double yMax() const
    {
        return _yMax;
    }

    double yMin() const
    {
        return _yMin;
    }

    /// Push all four sides outward by 'amount' >=0.
    void expand( T amount )
    {
        _xMin -= amount;
        _yMin -= amount;
        _xMax += amount;
        _yMax += amount;
    }

    /// Push all four sides inward by 'amount' >=0, possibly resulting in a zero-size box centered
    /// at the original center.
    void shrink( T amount )
    {
        const auto originalXCenter = ( _xMin + _xMax ) / 2.0;
        _xMin += amount;
        _xMax -= amount;
        if( _xMin > _xMax ) {
            _xMin = _xMax = originalXCenter;
        }

        const auto originalYCenter = ( _yMin + _yMax ) / 2.0;
        _yMin += amount;
        _yMax -= amount;
        if( _yMin > _yMax ) {
            _yMin = _yMax = originalYCenter;
        }
    }

    T size() const
    {
        return (_xMax-_xMin)*(_yMax-_yMin);
    }

    /// 'u' and 'v' in [0,1]
    Vector2 posFromUV( double u, double v ) const
    {
        // Can't include mathUtility::lerp here or I get a circular dependency.
        return Vector2{
            _xMin + ( _xMax - _xMin ) * u,
            _yMin + ( _yMax - _yMin ) * v };
    }

    Vector2 posFromUV( const Vector2& uv ) const
    {
        return posFromUV( uv.x(), uv.y() );
    }

    Vector2 uvFromPos( const Vector2& pos ) const
    {
        const auto x = ( pos.x() - _xMin ) / ( _xMax - _xMin );
        const auto y = ( pos.y() - _yMin ) / ( _yMax - _yMin );
        return Vector2{ x, y };
    }

    // This assumes that upper limits are _exclusive_, not _inclusive_!  (Matters for
    // integral T).
    T widthExclusive() const
    {
        return _xMax-_xMin;
    }
    T heightExclusive() const
    {
        return _yMax-_yMin;
    }

    TCoord center() const
    {
        return TCoord( ( _xMax + _xMin ) * 0.5, ( _yMax + _yMin ) * 0.5 );
    }

    bool inXRange(double x) const
    {
        return x>=_xMin && x<=_xMax;
    }

    bool inYRange(double y) const
    {
        return y>=_yMin && y<=_yMax;
    }

    TCoord constrain( const TCoord& coord ) const
    {
        TCoord constrained = coord;
        if( constrained.x() < _xMin ) constrained.setX( _xMin );
        if( constrained.x() > _xMax ) constrained.setX( _xMax );
        if( constrained.y() < _yMin ) constrained.setY( _yMin );
        if( constrained.y() > _yMax ) constrained.setY( _yMax );
        return constrained;
    }

    bool contains(const TCoord& coord) const
    {
        T x = coord.x();
        T y = coord.y();

        return x>=_xMin && x<=_xMax && y>=_yMin && y<=_yMax;
    }

    bool contains( const BoundingBox& box ) const
    {
        return contains( box.topLeft() )
            && contains( box.topRight() )
            && contains( box.bottomLeft() )
            && contains( box.bottomRight() );
    }

    //meant for the scanline processing in SwathPaint
    bool intersectsScanline(double yEquals) const
    {
        return yEquals<=_yMax && yEquals>=_yMin;
    }

    bool intersects(const BoundingBox<T,TCoord>& other) const
    {
        return _xMin <= other._xMax &&
               _xMax >= other._xMin &&
               _yMin <= other._yMax &&
               _yMax >= other._yMin;
    }

    void growToContain(const BoundingBox<T,TCoord>& other)
    {
        addPoint(TCoord(other._xMin,other._yMin));
        addPoint(TCoord(other._xMax,other._yMax));
    }

    T maxDim() const
    {
        return std::max< T >( widthExclusive(), heightExclusive() );
    }

    T minDim() const
    {
        return std::min< T >( widthExclusive(), heightExclusive() );
    }

    T avgDim() const
    {
        return ( widthExclusive() + heightExclusive() ) / T{ 2 };
    }

    BoundingInterval<T> xInterval() const
    {
        return BoundingInterval<T>(_xMin,_xMax);
    }

    BoundingInterval<T> yInterval() const
    {
        return BoundingInterval<T>(_yMin,_yMax);
    }

    /// Return the position corresponding to an input from (0,0)->(1,1).
    Vector2 fromNormalizedPos( const TCoord& normalized ) const
    {
        return TCoord( _xMin + normalized.x() * ( _xMax - _xMin ),
                       _yMin + normalized.y() * ( _yMax - _yMin ) );
    }

    /// Constrain output to [0,1].
    Vector2 normalize( const TCoord& p ) const
    {
        const auto fX = ( p.x() - _xMin ) / ( _xMax - _xMin );
        const auto fY = ( p.y() - _yMin ) / ( _yMax - _yMin );
        return Vector2
        {
            std::clamp< double >( fX, 0., 1. ),
            std::clamp< double >( fY, 0., 1. )
        };
    }

    void addPoint(const TCoord& point)
    {
        T x = point.x();
        T y = point.y();
        if(x<_xMin)
        {
            _xMin=x;
        }
        if(x>_xMax)
        {
            _xMax=x;
        }
        if(y<_yMin)
        {
            _yMin=y;
        }
        if(y>_yMax)
        {
            _yMax=y;
        }
    }

    static boost::optional< BoundingBox > intersection( const BoundingBox& a, const BoundingBox& b )
    {
        const boost::optional< BoundingInterval< T > > xIntersected = BoundingInterval< T >::intersection( a.xInterval(), b.xInterval() );
        const boost::optional< BoundingInterval< T > > yIntersected = BoundingInterval< T >::intersection( a.yInterval(), b.yInterval() );

        if( xIntersected.is_initialized() && yIntersected.is_initialized() ) {
            const BoundingInterval< T >& xInterval = xIntersected.value();
            const BoundingInterval< T >& yInterval = yIntersected.value();
            return BoundingBox( xInterval, yInterval );
        } else {
            return boost::none;
        }
    }

    /// Create an invalid instance which should not be used until at least one point has been added.
    static BoundingBox aroundPoint( const TCoord& center, T radius )
    {
        BoundingBox toReturn;
        toReturn._xMin = center.x() - radius;
        toReturn._xMax = center.x() + radius;
        toReturn._yMin = center.y() - radius;
        toReturn._yMax = center.y() + radius;
        return toReturn;
    }

    static BoundingBox aroundPoint( const TCoord& center, T width, T height )
    {
        BoundingBox toReturn;
        toReturn._xMin = center.x() - width / 2;
        toReturn._xMax = center.x() + width / 2;
        toReturn._yMin = center.y() - height / 2;
        toReturn._yMax = center.y() + height / 2;
        return toReturn;
    }

    static double diagonalLength( const BoundingBox& box )
    {
        return ( box.topLeft() - box.bottomRight() ).length();
    }

    static BoundingBox uvSpace()
    {
        return BoundingBox( TCoord( 0, 0 ), TCoord( 1, 1 ) );
    }

private:

    T _xMin;
    T _xMax;
    T _yMin;
    T _yMax;
};

typedef BoundingBox<double,Vector2> BoundingBoxd;
typedef BoundingBox<int,IntCoord> BoundingBoxi;

BoundingBoxi boundingBoxFloatToInt(const BoundingBoxd& b);
BoundingBoxd boundingBoxIntToFloat(const BoundingBoxi& b);

} // core

#endif // #include guard
