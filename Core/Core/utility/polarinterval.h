#ifndef CORE_POLARINTERVAL_H
#define CORE_POLARINTERVAL_H

#include <boost/math/constants/constants.hpp>

namespace core {

/// An alternative to BoundingIntervald meant to handle 2pi periodicity.
class PolarInterval
{
public:
    PolarInterval( double radians, double modCeil = 2.0 * boost::math::constants::pi< double >() );
    void add( double radians, bool counterclockwiseFromLastAddedAngle );
    double minRadians() const;
    double maxRadians() const;
    bool full() const;
    double length() const;
private:    
    double _modCeil;
    double _min;
    double _max;
    double _lastAngleAdded;
    bool _full;
};

} // core

#endif // #include guard
