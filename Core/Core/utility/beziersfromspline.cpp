#include <utility/beziersfromspline.h>

namespace core {

BeziersFromSpline::BeziersFromSpline( const Spline& spline )
{
    // Is 'spline' already a single Bezier?
    if( spline.numBezierCurves( true ) == 1 ) {
        _beziers.push_back( &spline );
        _tStarts = { 0. };
    } else {
        const auto controlPoints = spline.breakIntoBCurves( _tStarts );
        const int numCurves = static_cast< int >( controlPoints.size() );
        _beziers.resize( numCurves );
        _createdBeziers.resize( numCurves );
        for( int c = 0; c < numCurves; c++ ) {
            _createdBeziers[ c ] = std::make_unique< Spline >( spline.degree(), controlPoints[ c ] );
            _beziers[ c ] = _createdBeziers[ c ].get();
        }
    }
}

const std::vector< double >& BeziersFromSpline::tStarts() const
{
    return _tStarts;
}

const BeziersFromSpline::BezierHandles& BeziersFromSpline::beziers() const
{
    return _beziers;
}

} // core
