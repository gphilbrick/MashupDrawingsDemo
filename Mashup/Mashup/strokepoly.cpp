#include <strokepoly.h>

#include <strokeback.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/model/curveback.h>
#include <Core/model/lineback.h>
#include <Core/model/stroketools.h>

#include <PrintCurves/miteredcurve.h>

#include <Core/utility/mathutility.h>

namespace mashup {

using Pos = core::model::Pos;
using Seg = core::model::Seg;

namespace {

/// Return whether 'p' is within 'buffer' of lying on 'a'->'b'.
bool rightOnSeg( const Pos& p, const Pos& a, const Pos& b, double buffer )
{
    return core::mathUtility::distToLineSegment( p, a, b ) <= buffer;
}

/// 'a', 'b', and 'c' in one winding order or the other. Use 'buffer' to err on side of false positive
/// when 'point' is right on one of the edges.
bool pointInsideTri( const Pos& point, const Pos& a, const Pos& b, const Pos& c, double buffer )
{
    // <X> Using the dot-product principle shown at
    // http://totologic.blogspot.com/2014/01/accurate-point-in-triangle-test.html

    auto abNorm = b - a;
    abNorm.turnPerpendicular( false, true );
    if( Pos::dot( abNorm, a - point ) < 0 ) {
        if( !rightOnSeg( point, a, b, buffer ) ) {
            return false;
        }
    }

    auto bcNorm = c - b;
    bcNorm.turnPerpendicular( false, true );
    if( Pos::dot( bcNorm, b - point ) < 0 ) {
        if( !rightOnSeg( point, b, c, buffer ) ) {
            return false;
        }
    }

    auto caNorm = a - c;
    caNorm.turnPerpendicular( false, true );
    if( Pos::dot( caNorm, c - point ) < 0 ) {
        if( !rightOnSeg( point, a, c, buffer ) ) {
            return false;
        }
    }

    return true;
}

void tFromStroke( const Stroke& s, size_t numPointsAskedFor, std::vector< double >& storeT )
{
    if( core::model::isSimpleSegStroke( s ) ) {
        // Make debugging easier.
        storeT = { 0., 1. };
        return;
    }

    if( s.curve().degree() == 1 ) {
        // If 's' is a box (or something similar), make sure to perfectly
        // capture its corners in 'storeT' so that said corners don't get
        // slightly chopped off in the approximation.
        storeT = s.curve().tForPolylineApprox( { 0., 1. }, numPointsAskedFor );
    } else {
        storeT.resize( numPointsAskedFor );
        for( size_t i = 0; i < numPointsAskedFor; i++ ) {
            storeT[ i ] = F_FROM_I( i, numPointsAskedFor );
        }
    }
}

} // unnamed

StrokePoly::StrokePoly()
    : stroke( nullptr )
    , killForEachSeg( false )
{
}

StrokePoly::StrokePoly( const Stroke& s, size_t numPoints )
{
    initFrom( s, numPoints );
}

bool StrokePoly::outlineCrosses( const Seg& seg ) const
{
    bool hitFound = false;
    Pos unused;
    forEachSeg( [ & ]( const Seg& outlineSeg, const Normal&, double, double )
    {
        if( core::mathUtility::segmentsIntersect( seg, outlineSeg, unused ) ) {
            hitFound = true;
            killForEachSeg = true;
        }
    } );
    return hitFound;
}

bool StrokePoly::contains( const Pos& p ) const
{
    const auto len = pointsPerSide();
    if( len < 2 ) {
        return false;
    }

    for( size_t i = 0; i < len - 1; i++ ) {
        const auto& lA = sides[ Left ][ i ];
        const auto& lB = sides[ Left ][ i + 1 ];
        const auto& rA = sides[ Right ][ i ];
        const auto& rB = sides[ Right ][ i + 1 ];

        core::model::BoundingBox bounds;
        bounds.addPoint( lA );
        bounds.addPoint( lB );
        bounds.addPoint( rA );
        bounds.addPoint( rB );

        // Don't want this to be too large in case of tris representing long, thin strokes.
        const auto buffer = bounds.minDim() * 1e-2;

        bounds.expand( buffer );
        if( bounds.contains( p ) ) {
            if( pointInsideTri( p, lA, lB, rA, buffer ) ) {
                return true;
            }
            if( pointInsideTri( p, rA, lB, rB, buffer ) ) {
                return true;
            }
        }
    }
    return false;
}

/// Return a point representing moving 'seekT' (in [0,1
Pos StrokePoly::onSide( double seekT, StrokeSide sideIdx ) const
{
    const auto len = pointsPerSide();
    if( len < 2 ) {
        THROW_UNEXPECTED;
    }

    const auto& side = sides[ sideIdx ];
    for( size_t i = 0; i < len - 1; i++ ) {
        const auto& prevT = t[ i ];
        const auto& nextT = t[ i + 1 ];
        if( nextT >= seekT ) {
            return Pos::lerp( side[ i ],
                              side[ i + 1 ],
                              ( seekT - prevT ) / ( nextT - prevT ) );
        }
    }
    return side.back();
}

void StrokePoly::initFrom( const Stroke& s, size_t numPointsAskedFor )
{
    stroke = &s;

    if( s.zeroLength() ) {
        // Leave 's' out of crossing calculations.
        return;
    }

    if( stroke->closed() ) {
        init_closed( numPointsAskedFor );
    } else {
        init_open( numPointsAskedFor );
    }
}

void StrokePoly::init_open( size_t numPointsAskedFor )
{
    const auto& s = *stroke;

    if( s.zeroLength() ) {
        // Leave 's' out of crossing calculations.
        return;
    }
    const auto& curve = s.curve();

    tFromStroke( s, numPointsAskedFor, t );
    const auto numPoints = t.size();

    for( size_t i = 0; i < NumSides; i++ ) {
        sides[ i ].resize( numPoints );
        sideNormals[ i ].resize( numPoints - 1 );
    }

    for( size_t i = 0; i < numPoints; i++ ) {
        const auto t_i = t[ i ];

        const auto onS = curve.position( t_i );
        const auto w = s.width( t_i );
        auto dir = curve.derivative( t_i );
        dir.normalize();

        // Note that I'm mentally modeling w.r.t. origin at top-left.
        auto toLeft = dir.perpendicular( false, true );

        for( size_t side = 0; side < NumSides; side++ ) {
            sides[ side ][ i ] = onS + toLeft * w * 0.5 * ( side == Left ? 1. : -1. );
            bounds.addPoint( sides[ side ][ i ] );
            if( i > 0 ) {
                sideNormals[ side ][ i - 1 ] = sides[ side ][ i ] - sides[ side ][ i - 1 ];
                sideNormals[ side ][ i - 1 ].turnPerpendicular( side == Right, true );
                sideNormals[ side ][ i - 1 ].normalize();
            }
        }
    }

    capNormal_T0 = sides[ Left ].front() - sides[ Right ].front();
    capNormal_T0->turnPerpendicular( false, true );
    capNormal_T0->normalize();

    capNormal_T1 = sides[ Left ].back() - sides[ Right ].back();
    capNormal_T1->turnPerpendicular( true, true );
    capNormal_T1->normalize();
}

void StrokePoly::init_closed( size_t numPointsAskedFor )
{
    const auto& s = *stroke;

    tFromStroke( s, numPointsAskedFor, t );
    const auto numP = t.size();
    if( numP < 2 ) {
        THROW_UNEXPECTED;
    }

    const auto& curve = s.curve();
    std::vector< core::Vector2 > pos( numP );
    std::vector< double > baseWidths( numP );

    for( size_t i = 0; i < numP; i++ ) {
        const auto& tVal = t[ i ];
        pos[ i ] = curve.position( tVal );
        baseWidths[ i ] = s.width( tVal );
    }

    printCurves::miteredOffsetSamples(
        pos,
        baseWidths,
        baseWidths,
        sides[ StrokeSide::Left ],
        sides[ StrokeSide::Right ] );

    for( int side = 0; side < StrokeSide::NumSides; side++ ) {
        sideNormals[ side ].resize( numP - 1 );
        for( size_t i = 0; i < numP - 1; i++ ) {
            bounds.addPoint( sides[ side ][ i ] );

            const auto& a = sides[ side ][ i ];
            const auto& b = sides[ side ][ i + 1 ];
            auto norm = b - a;
            norm.turnPerpendicular( side == StrokeSide::Right, true );
            norm.normalize();
            sideNormals[ side ][ i ] = norm;
        }
        bounds.addPoint( sides[ side ].back() );
    }
}

void StrokePoly::forEachSeg( std::function< void( const Seg&, const Normal&, double, double ) > f_seg_norm_tA_tB ) const
{
    killForEachSeg = false;

    if( !participates() ) {
        THROW_RUNTIME( "Can't call on 'non-participating' StrokePoly" );
    }

    // Apply 'f...' to all the segments of each side.
    for( size_t i = 0; i < NumSides && !killForEachSeg; i++ ) {
        const auto& side = sides[ i ];
        const auto& sideNorms = sideNormals[ i ];
        for( size_t i = 0; i < side.size() - 1; i++ ) {
            const auto tA = t[ i ];
            const auto tB = t[ i + 1 ];
            const Seg seg{ side[ i ], side[ i + 1 ] };
            f_seg_norm_tA_tB( seg, sideNorms[ i ], tA, tB );
        }
    }
    if( killForEachSeg ) {
        return;
    }

    // Include the "caps".
    if( !closed() ) {
        f_seg_norm_tA_tB( Seg{ sides[ Left ].front(), sides[ Right ].front() },
                         *capNormal_T0,
                         0.,
                         0. );
        if( killForEachSeg ) {
            return;
        }
        f_seg_norm_tA_tB( Seg{ sides[ Left ].back(), sides[ Right ].back() },
                         *capNormal_T1,
                         1.,
                         1. );
    }
}

size_t StrokePoly::pointsPerSide() const
{
    return sides[ Left ].size();
}

bool StrokePoly::participates() const
{
    return pointsPerSide() > 0;
}

std::vector< double > StrokePoly::hitTs( const core::model::Polyline& hitter ) const
{
    std::vector< double > ret;
    if( hitter.size() <= 1 ) {
        return ret;
    }

    for( size_t i = 0; i < hitter.size() - 1; i++ ) {
        const auto& a_hitter = hitter[ i ];
        const auto& b_hitter = hitter[ i + 1 ];
        const core::model::BoundingBox segBounds( a_hitter, b_hitter );
        if( !bounds.intersects( segBounds ) ) {
            continue;
        }

        forEachSeg(
        [ & ]( const core::model::Seg& ab, const Normal&, double tA, double tB )
        {
            core::model::Pos hit;
            if( core::mathUtility::segmentsIntersect( a_hitter, b_hitter, ab.a, ab.b, hit ) ) {
                ret.push_back( core::mathUtility::lerp( tA, tB, ab.t( hit ) ) );
            }
        } );
    }
    return ret;
}

bool StrokePoly::hitsAtAll( const core::model::Polyline& hitter ) const
{
    if( hitter.size() <= 1 ) {
        return false;
    }

    for( size_t i = 0; i < hitter.size() - 1; i++ ) {
        const auto& a_hitter = hitter[ i ];
        const auto& b_hitter = hitter[ i + 1 ];
        const core::model::BoundingBox segBounds( a_hitter, b_hitter );
        if( !bounds.intersects( segBounds ) ) {
            continue;
        }

        bool hitFound = false;
        forEachSeg(
        [ & ]( const core::model::Seg& ab, const Normal&, double, double )
        {
            core::model::Pos hit;
            if( core::mathUtility::segmentsIntersect( a_hitter, b_hitter, ab.a, ab.b, hit ) ) {
                killForEachSeg = true;
                hitFound = true;
            }
        } );
        if( hitFound ) {
            return true;
        }
    }
    return false;
}

bool StrokePoly::closed() const
{
    return !capNormal_T0.is_initialized();
}

} // mashup
