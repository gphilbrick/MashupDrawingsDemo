#include <substroke.h>

#include <strokeback.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/model/curveback.h>

#include <Core/utility/mathutility.h>

namespace mashup {

Substroke::Substroke()
    : stroke( nullptr )
    , t{ 0, 0 }
{}

Substroke::Substroke( const Stroke& s, double tA, double tB )
    : stroke( &s )
    , t{ tA, tB }
{}

Substroke Substroke::reverse() const
{
    return Substroke( *stroke, t[ 1 ], t[ 0 ] );
}

bool Substroke::tIncreasing() const
{
    return t[ 1 ] > t[ 0 ];
}

bool Substroke::operator==( const Substroke& b ) const
{
    return stroke == b.stroke && t[ 0 ] == b.t[ 0 ] && t[ 1 ] == b.t[ 1 ];
}

bool Substroke::operator!=( const Substroke& b ) const
{
    return !( *this == b );
}

bool Substroke::compare_standard( const Substroke& a, const Substroke& b )
{
    if( a.stroke == b.stroke ) {
        if( a.t[ 0 ] == b.t[ 0 ] ) {
            return a.t[ 1 ] < b.t[ 1 ];
        } else {
            return a.t[ 0 ] < b.t[ 0 ];
        }
    } else {
        return a.stroke < b.stroke;
    }
}

std::unique_ptr< Stroke > Substroke::asStroke() const
{
    return stroke->strokeInterval( t[ 0 ], t[ 1 ] );
}

core::model::Pos Substroke::endpoint( bool endOrStart ) const
{
    return stroke->curve().position( endOrStart ? t[ 1 ] : t[ 0 ] );
}

double Substroke::endWidth( bool endOrStart ) const
{
    return stroke->width( endOrStart ? t[ 1 ] : t[ 0 ] );
}

core::model::Pos Substroke::endDirNormalized( bool endOrStart ) const
{
    auto dir = stroke->curve().derivative( t[ endOrStart ? 1 : 0 ] );
    dir.normalize();
    return tIncreasing() ? dir : dir * -1.;
}

bool Substroke::includesStrokeEndpoint( bool atEndOrStart ) const
{
    const auto& tVal = t[ atEndOrStart? 1 : 0 ];
    return tVal == 0. || tVal == 1.;
}

bool Substroke::contains( double t_ ) const
{
    if( tIncreasing() ) {
        return t_ >= t[ 0 ] && t_ <= t[ 1 ];
    } else {
        return t_ >= t[ 1 ] && t_ <= t[ 0 ];
    }
}

Substroke Substroke::interval( double fA, double fB ) const
{
    return Substroke
    {
        *stroke,
        core::mathUtility::lerp( t[ 0 ], t[ 1 ], fA ),
        core::mathUtility::lerp( t[ 0 ], t[ 1 ], fB ),
    };
}

double Substroke::tFromF( double f ) const
{
    return core::mathUtility::lerp( t[ 0 ], t[ 1 ], f );
}

double Substroke::f( double tStroke ) const
{
    if( core::mathUtility::closeEnough( t[ 0 ], t[ 1 ] ) ) {
        return 0.;
    }
    return std::clamp( ( tStroke - t[ 0 ] ) / ( t[ 1 ] - t[ 0 ] ), 0., 1. );
}

void Substroke::nonFlippingTrim( double trimT, bool start )
{
    if( t[ 0 ] == t[ 1 ] ) {
        return;
    }

    const bool tInc = tIncreasing();

    const auto throwIf = []( bool cond )
    {
        if( cond ) {
            THROW_RUNTIME( "trimT would flip or set length=0" );
        }
    };

    if( start ) {
        // Cutting off part of start of 'this'.
        if( tInc ) {
            // Increase t[ 0 ], but not up to t[ 1 ].
            throwIf( trimT >= t[ 1 ] );
            t[ 0 ] = std::max( t[ 0 ], trimT );
        } else {
            // Decrease t[ 0 ], but not down to t[ 1 ].
            throwIf( trimT <= t[ 1 ] );
            t[ 0 ] = std::min( t[ 0 ], trimT );
        }
    } else {
        // Cutting off part of end of 'this'.
        if( tInc ) {
            // Decrease t[ 1 ], but not down to t[ 0 ]
            throwIf( trimT <= t[ 0 ] );
            t[ 1 ] = std::min( t[ 1 ], trimT );
        } else {
            // Increase t[ 1 ], but not up to t[ 0 ]
            throwIf( trimT >= t[ 0 ] );
            t[ 1 ] = std::max( t[ 1 ], trimT );
        }
    }

    if( tIncreasing() != tInc ) {
        THROW_UNEXPECTED;
    }
}

} // mashup
