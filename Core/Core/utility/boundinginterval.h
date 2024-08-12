#ifndef CORE_BOUNDINGINTERVAL_H
#define CORE_BOUNDINGINTERVAL_H

#include <boost/optional.hpp>

#include <array>
#include <vector>

namespace core {

template< typename T >
class BoundingInterval
{
public:    
    using Value = T;

    BoundingInterval()
    {
        _min = std::numeric_limits< T >::max();
        _max = std::numeric_limits< T >::lowest();
    }

    explicit BoundingInterval( T a ) : _min( a ), _max( a )
    {
    }

    BoundingInterval(T a, T b)
    {
        _min = (a<b) ? a : b;
        _max = (a<b) ? b : a;
    }

    explicit BoundingInterval( const std::array< T, 2 >& args ) : BoundingInterval( args[ 0 ], args[ 1 ] )
    {
    }

    bool operator == ( const BoundingInterval& other ) const
    {
        return _min == other._min && _max == other._max;
    }

    bool operator != ( const BoundingInterval& other ) const
    {
        return !( *this == other );
    }

    T min() const { return _min; }
    T max() const { return _max; }

    bool intersects(const BoundingInterval<T>& other) const
    {
        return _min <= other._max &&
               _max >= other._min;
    }

    /// Adjust the start and end so that they fall inside 'to' the same way they currently fall inside 'from'. For instance,
    /// if this interval is [0.5, 0.75], 'from' is [0,1], and 'to' is [100,200], then make this interval [150,175]. Only
    /// call this if T is floating point.
    void remap( const BoundingInterval< T >& from, const BoundingInterval< T >& to )
    {
        _min = remap( _min, from, to );
        _max = remap( _max, from, to );
    }

    void add( T val )
    {
        if( val < _min ) {
            _min = val;
        }
        if( val > _max ) {
            _max = val;
        }
    }

    /// Return 'toRemap' (presumed but not required to fall within 'from') mapped from 'from' to 'to'. For instance, if
    /// 'toRemap' is 0.5, 'from' is [0,1], and 'to' is [100,200], then return 150. Only call this if T is floating point.
    static double remap( double toRemap, const BoundingInterval< T >& from, const BoundingInterval< T >& to )
    {
        const double fromRange = from._max - from._min;
        const double toRange = to._max - to._min;
        return ( toRemap - from._min ) / fromRange * toRange + to._min;
    }

    /// If there is an intersection between 'a' and 'b', return it; otherwise, return boost::none.
    static boost::optional< BoundingInterval > intersection( const BoundingInterval& a, const BoundingInterval& b )
    {
        if(!a.intersects(b))
        {
            return boost::none;
        }
        else
        {
            T min = 0, max = 0;
            if( a._min < b._min )
            {
                min = b._min;
            }
            else
            {
                min = a._min;
            }

            if( a._max > b._max )
            {
                max = b._max;
            }
            else
            {
                max = a._max;
            }
            return BoundingInterval( min, max );
        }
    }

    bool contains(T val) const
    {
        return val>=_min && val<=_max;
    }

    void expandToContain(const BoundingInterval<T>& other)
    {
        if(other._min<_min)
        {
            _min=other._min;
        }
        if(other._max>_max)
        {
            _max=other._max;
        }
    }

    void bisect(BoundingInterval<T>& a, BoundingInterval<T>& b) const
    {
        T mid = midpoint();
        a = BoundingInterval<T>(_min,mid);
        b = BoundingInterval<T>(mid,_max);
    }

    T midpoint() const
    {
        return (_max+_min)/2.0;
    }

    T length() const
    {
        return _max - _min;
    }

    /// Use 'f' in [0,1] to linear interpolate between '_min' and '_max'.
    T lerp( T f ) const
    {
        static_assert( std::is_floating_point<T>::value, "T must be floating point" );
        return _min + ( _max - _min ) * f;
    }

    bool zeroLength() const
    {
        return _max == _min;
    }

private:
    T _min;
    T _max;
};

typedef BoundingInterval<int> BoundingIntervali;
typedef BoundingInterval<double> BoundingIntervald;

/// Return the 0-2 intervals resulting from removing 'remove' from 'removeFrom', ignoring any resulting intervals of length < 'epsilon'.
std::vector< BoundingIntervald > removeBoundingInterval( const BoundingIntervald& removeFrom, const BoundingIntervald& remove, double epsilon );

} // core

#endif // #include guard
