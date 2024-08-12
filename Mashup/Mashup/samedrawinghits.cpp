#include <samedrawinghits.h>

#include <substroke.h>
#include <topology/crossing.h>

namespace mashup {

bool SameDrawingHits::Hit::operator < ( const Hit& b ) const
{
    if( t == b.t ) {
        if( other == b.other ) {
            return tOther < b.tOther;
        } else {
            return other < b.other;
        }
    } else {
        return t < b.t;
    }
}

void SameDrawingHits::addHit( StrokeHandle a, StrokeHandle b, double tA, double tB )
{
    Hit aHit;
    aHit.other = b;
    aHit.t = tA;
    aHit.tOther = tB;
    _strokeToHits[ a ].emplace( aHit );

    Hit bHit;
    bHit.other = a;
    bHit.t = tB;
    bHit.tOther = tA;
    _strokeToHits[ b ].emplace( bHit );
}

void SameDrawingHits::clear()
{
    _strokeToHits.clear();
}

boost::optional< double > SameDrawingHits::firstOrLastHit(
    const Substroke& ss, bool firstOrLast, const topology::Crossing& ignoreIn ) const
{
    const auto it = _strokeToHits.find( ss.stroke );
    if( it == _strokeToHits.end() ) {
        return boost::none;
    }

    const auto& hits = it->second;

    boost::optional< double > ret;

    const bool minOrMax = ss.tIncreasing() == firstOrLast;

    for( const auto& hit : hits ) {
        if( !ss.contains( hit.t ) ) {
            continue;
        }

        if( ignoreIn.isPartOf( hit.other, hit.tOther ) ) {
            continue;
        }

        if( ret ) {
            ret = minOrMax
                ? std::min( *ret, hit.t )
                : std::max( *ret, hit.t );
        } else {
            ret = hit.t;
        }
    }

    return ret;
}

} // mashup
