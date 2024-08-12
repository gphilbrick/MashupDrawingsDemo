#include <model/stroke.h>

#include <exceptions/runtimeerror.h>
#include <model/curveback.h>
#include <model/stroketools.h>

#include <utility/casts.h>
#include <utility/curveinterval.h>
#include <utility/mathutility.h>

#include <boost/numeric/conversion/cast.hpp>

#include <iomanip>
#include <sstream>

namespace core {
namespace model {

Stroke::Stroke( double width ) : Stroke( constWidthCurve( width ) )
{
}

Stroke::Stroke( const WidthCurve& width ) : _width( std::make_unique< WidthCurve >( width ) )
{
}

Stroke::Stroke( std::unique_ptr< WidthCurve >&& width ) : _width( std::move( width ) )
{
}

std::unique_ptr< Stroke > Stroke::clone() const
{
    auto ret = std::make_unique< Stroke >( _width->clone() );
    ret->_curve = _curve->clone();
    return ret;
}

std::unique_ptr< Stroke > Stroke::strokeInterval( double tStart, double tEnd ) const
{
    const CurveInterval interval( tStart, tEnd );
    return strokeInterval( interval );
}

std::unique_ptr< Stroke > Stroke::strokeInterval( const CurveInterval& interval ) const
{
    // Cut up the width curve.
    auto widthInterval = _width->extractCurveForTInterval( interval );
    auto ret = std::make_unique< Stroke >( std::move( widthInterval ) );
    ret->setCurve( _curve->extractCurveForTInterval( interval ) );
    return ret;
}

std::unique_ptr< Stroke > Stroke::reverse() const
{
    auto reversedWidthCurve = _width->reverseCopy();
    auto ret = std::make_unique< Stroke >( std::move( reversedWidthCurve ) );
    ret->setCurve( _curve->reverseCopy() );
    return ret;
}

bool Stroke::closed() const
{
    return _curve && _curve->endpointsEqual();
}

const Curve& Stroke::curve() const
{
    return *_curve;
}

void Stroke::setCurve( const Curve& curve )
{
    _curve = std::make_unique< Curve >( curve );
}

void Stroke::setCurve( UniqueCurve&& curveToOwn )
{
    _curve = std::move( curveToOwn );
}

BoundingBoxd Stroke::boundingBox() const
{
    auto toReturn = _curve->boundingBox();
    toReturn.expand( 0.5 * maxWidth() );
    return toReturn;
}

bool Stroke::zeroLength() const
{
    return _curve->degree() == 1
        && _curve->controlPoints().size() == 2
        && _curve->startPosition() == _curve->endPosition();
}

double Stroke::width( double t ) const
{
    return std::max( 0.0, _width->position( t ).y() );
}

double Stroke::maxWidth() const
{
    double toReturn = 0.0;
    const auto& control = _width->controlPoints();
    for( const auto& p : control ) {
        if( p.y() > toReturn ) {
            toReturn = p.y();
        }
    }
    return toReturn;
}

const Stroke::WidthCurve& Stroke::widthCurve() const
{
    return *_width;
}

} // model
} // core
