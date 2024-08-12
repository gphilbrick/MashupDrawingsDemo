#include <utility/polarinterval.h>

namespace core {

PolarInterval::PolarInterval( double radians, double modCeil ) : _full( false ), _modCeil( modCeil )
{
    const double normalized = std::fmod( radians, _modCeil );
    _min = normalized;
    _max = normalized;
    _lastAngleAdded = normalized;
}

double PolarInterval::length() const
{
    return _max - _min;
}

void PolarInterval::add( double radiansUnnormalized, bool counterclockwiseFromLastAddedAngle )
{
    if( _full ) {
        return;
    }

    double radians = std::fmod( radiansUnnormalized, _modCeil );

    if( counterclockwiseFromLastAddedAngle ) {
        while( radians < _lastAngleAdded ) {
            radians += _modCeil;
        }
        double copy = radians;
        while( copy > _lastAngleAdded ) {
            radians = copy;
            copy -= _modCeil;
        }
        if( radians > _max ) {
            _max = radians;
        }
    } else {
        while( radians > _lastAngleAdded ) {
            radians -= _modCeil;
        }
        double copy = radians;
        while( copy < _lastAngleAdded ) {
            radians = copy;
            copy += _modCeil;
        }
        if( radians < _min ) {
            _min = radians;
        }
    }
    _lastAngleAdded = radians;

    if( _max - _min >= _modCeil ) {
        _full = true;
        _min = 0.0;
        _max = _modCeil;
    }
}

bool PolarInterval::full() const
{
    return _full;
}

double PolarInterval::minRadians() const
{
    return _min;
}

double PolarInterval::maxRadians() const
{
    return _max;
}

} // core
