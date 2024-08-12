#include <utility/curveinterval.h>

#include <utility/mathutility.h>

namespace core {

CurveInterval::CurveInterval() : CurveInterval( 0.0, 1.0, true )
{
}

CurveInterval::CurveInterval( double tStart, double tEnd ) :
    _tStart( tStart ),
    _tEnd( tEnd ),
    _tIncreasing( tStart <= tEnd )
{
}

CurveInterval::CurveInterval( double tStart, double tEnd, bool tIncreasing ) :
    _tStart( tStart),
    _tEnd( tEnd ),
    _tIncreasing( tIncreasing )
{
}

CurveInterval::CurveInterval( const CurveInterval& other )
{
    _tStart = other._tStart;
    _tEnd = other._tEnd;
    _tIncreasing = other._tIncreasing;
}

CurveInterval& CurveInterval::operator=( const CurveInterval& other )
{
    _tStart = other._tStart;
    _tEnd = other._tEnd;
    _tIncreasing = other._tIncreasing;
    return *this;
}

double CurveInterval::tStart() const
{
    return _tStart;
}

double CurveInterval::tEnd() const
{
    return _tEnd;
}

bool CurveInterval::tIncreasing() const
{
    return _tIncreasing;
}

bool CurveInterval::wrapsAround() const
{
    return _tIncreasing != _tStart <= _tEnd;
}

double CurveInterval::t( double f ) const
{
    if( wrapsAround() ) {
        const auto firstFSection = _tIncreasing ? 1.0 - _tStart : _tStart;
        const auto secondFSection = _tIncreasing ? _tEnd : 1.0 - _tEnd;

        const auto fCutoff = ( firstFSection / ( firstFSection + secondFSection ) );
        double localF = f / fCutoff;
        if( localF < 1.0 ) {
            return _tIncreasing ? mathUtility::lerp( _tStart, 1.0, localF ) : mathUtility::lerp( _tStart, 0.0, localF );
        } else {
            localF = ( f - fCutoff ) / ( 1.0 - fCutoff );
            return _tIncreasing ? mathUtility::lerp( 0.0, _tEnd, localF ) : mathUtility::lerp( 1.0, _tEnd, localF );
        }
    } else {
        return mathUtility::lerp( _tStart, _tEnd, f );
    }
}

double CurveInterval::fFromT( double t ) const
{
    t = mathUtility::clamp( t, 0.0, 1.0 );
    if( wrapsAround() ) {
        const auto min = std::min( _tStart, _tEnd );
        const auto max = std::max( _tStart, _tEnd );
        const auto totalLength = min + 1.0 - max;
        if( _tIncreasing ) {
            if( t >= _tStart ) {
                return ( t - _tStart ) / totalLength;
            } else if( t <= _tEnd ) {
                return ( 1.0 - _tStart + t ) / totalLength;
            } else {
                return std::fabs( t - min ) < std::fabs( t - max ) ? 1.0 : 0.0;
            }
        } else {
            if( t >= _tEnd ) {
                return ( _tStart + ( 1.0 -  t ) ) / totalLength;
            } else if( t <= _tStart ) {
                return ( _tStart - t ) / totalLength;
            } else {
                return std::fabs( t - min ) < std::fabs( t - max ) ? 0.0 : 1.0;
            }
        }
    } else {
        return mathUtility::clamp( ( t - _tStart ) / ( _tEnd - _tStart ), 0.0, 1.0 );
    }
}

double CurveInterval::length() const
{
    if( wrapsAround() ) {
        if( tIncreasing() ) {
            return 1.0 - tStart() + tEnd();
        } else {
            return tStart() + 1.0 - tEnd();
        }
    } else {
        return std::abs( tStart() - tEnd() );
    }
}

} // core
