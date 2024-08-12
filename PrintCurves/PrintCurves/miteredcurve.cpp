#include <miteredcurve.h>

#include <Core/utility/bspline2.h>
#include <Core/utility/mathutility.h>
#include <Core/utility/wall.h>

namespace printCurves {

void c0TimesAndMiteredJoins(
    const core::BSpline2& spline,
    std::vector< double >& storeTimes,
    std::vector< core::Wall >& storeJoins )
{
    // This is similar to c0Times, but different enough that it's not really a brittle copy.
    const auto knots = spline.internalKnots();
    const auto degree = spline.degree();
    const auto& control = spline.controlPoints();

    storeTimes.clear();
    storeJoins.clear();

    int numMultiples = 0;
    for( size_t i = 0; i < knots.size(); i++ ) {
        const auto& knot = knots[ i ];

        // Try to ignore knot multiplicity at beginning and end of curve.
        if( knot == 0 ) {
            continue;
        }
        numMultiples++;

        const bool doneWithMultiplesRun =
                ( i == knots.size() - 1 )
                || ( !core::mathUtility::closeEnough( knot, knots[ i + 1 ] ) )
                || knot >= 1.0;

        if( doneWithMultiplesRun ) {
            // Do we have enough multiples to register a juncture?
            if( numMultiples >= degree ) {
                storeTimes.push_back( knot );

                // Imagine that the junction is a->b->c, where b is the control point for "0.5 0.5 0.5" in the case of
                // a spline with knot vector 0 0 0 0.5 0.5 0.5 1 1 1.
                const auto indexOfB = degree + i - numMultiples + 1; // really, the index of the _first_ occurrence of b (if knot multiplicity > degree, there
                // could be multiple copies of b, in effect).
                const auto& b = control[ indexOfB ];
                const auto& a = control[ indexOfB - 1 ];
                const auto& c = control[ indexOfB + ( numMultiples - degree ) + 1 ];

                // We want the direction heading into the junction and the direction heading out, which
                // means we want the control point _before_
                auto aToB = b - a;
                aToB.normalize();
                auto bToC = c - b;
                bToC.normalize();
                const core::Wall wall( b, aToB + bToC );
                storeJoins.push_back( wall );
            }

            numMultiples = 0;
        }
    }
}

void offsetSamplesForMiteredJoinRender(
    const core::BSpline2& spline,
    std::vector< std::vector< core::Vector2 > >& storeLeft,
    std::vector< std::vector< core::Vector2 > >& storeRight,
    SamplesPerInterval samplesPerInterval,
    ParamToWidth tToWidth )
{
    const LowLevelWidthFunctor widthFunc = [ &tToWidth ]( double t, const core::Vector2& )
    {
        return tToWidth( t );
    };
    return offsetSamplesForMiteredJoinRender(
        spline, storeLeft, storeRight, samplesPerInterval, widthFunc );
}

void offsetSamplesForMiteredJoinRender(
    const core::BSpline2& spline,
    std::vector< std::vector< core::Vector2 > >& storeLeft,
    std::vector< std::vector< core::Vector2 > >& storeRight,
    SamplesPerInterval samplesPerInterval,
    CanvasPosToWidth posToWidth )
{
    const LowLevelWidthFunctor widthFunc = [ &posToWidth ]( double, const core::Vector2& canvasPos )
    {
        return posToWidth( canvasPos );
    };
    return offsetSamplesForMiteredJoinRender(
        spline, storeLeft, storeRight, samplesPerInterval, widthFunc );
}

void offsetSamplesForMiteredJoinRender(
    const core::BSpline2& spline,
    std::vector< std::vector< core::Vector2 > >& storeLeft,
    std::vector< std::vector< core::Vector2 > >& storeRight,
    SamplesPerInterval samplesPerInterval,
    LowLevelWidthFunctor widthFunctor )
{
    // First come up with T values and keep track of indices of T values that represent corners of 'spline'.
    std::vector< double > t;
    std::vector< size_t > cornerT;
    size_t numIntervals = 1;
    {
        std::vector< double > c0Times;
        const std::vector< double > rawC0Times = core::BSpline2::c0Times( spline );
        double lastSeamTime = 0.0;
        for( size_t i = 0; i < rawC0Times.size(); i++ ) {
            if( !core::mathUtility::closeEnough( lastSeamTime, rawC0Times[ i ] ) ) {
                c0Times.push_back( rawC0Times[ i ] );
                lastSeamTime = c0Times.back();
            }
        }
        numIntervals = c0Times.size() + 1;

        for( size_t i = 0; i < numIntervals; i++ ) {
            const double tStart = i == 0 ? 0.0 : c0Times[ i - 1 ];
            const double tEnd = i == numIntervals - 1 ? 1.0 : c0Times[ i ];
            const core::BoundingIntervald tInterval{ tStart, tEnd };
            const auto numSamples = samplesPerInterval( tInterval );

            t.push_back( tStart );
            if( i > 0 ) {
                cornerT.push_back( t.size() - 1 );
            }

            for( size_t j = 1; j < numSamples; j++ ) {
                const auto f = F_FROM_I( j, numSamples + 1 ); // leave out f = 1.0 so we don't have redundant points.
                t.push_back( core::mathUtility::lerp( tStart, tEnd, f ) );
            }
        }
        t.push_back( 1.0 );
    }

    // Now get the spine positions and some widths.
    std::vector< core::Vector2 > spinePos( t.size() );
    std::vector< double > widths( t.size() );
    {
        for( size_t i = 0; i < t.size(); i++ ) {
            const auto& tVal = t[ i ];
            spinePos[ i ] = spline.position( tVal );
            widths[ i ] = widthFunctor( tVal, spinePos.back() );
        }
    }

    std::vector< core::Vector2 > allLeft, allRight;
    miteredOffsetSamples( spinePos, widths, allLeft, allRight );

    // Now split 'allLeft' and 'allRight' into 'storeLeft' and 'storeRight'.
    storeLeft.resize( numIntervals );
    storeRight.resize( numIntervals );
    for( size_t interval = 0; interval < numIntervals; interval++ ) {
        auto& left = storeLeft[ interval ];
        auto& right = storeRight[ interval ];
        left.clear();
        right.clear();

        const size_t firstPoint = interval == 0 ? 0 : cornerT[ interval - 1 ];
        const size_t lastPoint = interval == numIntervals - 1 ? spinePos.size() - 1 : cornerT[ interval ];
        left.insert( left.end(), allLeft.begin() + firstPoint, allLeft.begin() + lastPoint + 1 );
        right.insert( right.end(), allRight.begin() + firstPoint, allRight.begin() + lastPoint + 1 );
    }
}

void miteredOffsetSamples(
    const std::vector< core::Vector2 >& pos,
    const std::vector< double >& widths,
    std::vector< core::Vector2 >& storeLeft,
    std::vector< core::Vector2 >& storeRight,
    double maxRibFac )
{
    miteredOffsetSamples( pos, widths, widths, storeLeft, storeRight, maxRibFac );
}

void miteredOffsetSamples(
    const std::vector< core::Vector2 >& pos,
    const std::vector< double >& leftWidths,
    const std::vector< double >& rightWidths,
    std::vector< core::Vector2 >& storeLeft,
    std::vector< core::Vector2 >& storeRight,
    double maxRibFac )
{
    storeLeft.resize( pos.size() );
    storeRight.resize( pos.size() );
    if( pos.size() < 2 ) {
        return;
    }

    // <X> I'm following Artistic Silhouettes: A Hybrid Approach (https://dl.acm.org/doi/pdf/10.1145/340916.340920).

    std::vector< core::Vector2 > tangentDirs( pos.size() - 1 );
    for( size_t i = 0; i < pos.size() - 1; i++ ) {
        tangentDirs[ i ] = pos[ i + 1 ] - pos[ i ];
        tangentDirs[ i ].normalize();
    }

    const bool closedCurve = pos.front() == pos.back();

    std::vector< core::Vector2 > ribDirs( pos.size() );
    {
        if( closedCurve ) {
            auto comb = tangentDirs.back() + tangentDirs.front();
            comb.normalize();
            ribDirs.front() = comb;
            ribDirs.back() = comb;
            ribDirs.front().turnPerpendicular();
            ribDirs.back().turnPerpendicular();
        } else {
            ribDirs.front() = tangentDirs.front();
            ribDirs.back() = tangentDirs.back();
            ribDirs.front().turnPerpendicular();
            ribDirs.back().turnPerpendicular();
        }

        for( size_t i = 1; i < pos.size() - 1; i++ ) {
            const auto& tangentA = tangentDirs[ i - 1 ];
            const auto& tangentB = tangentDirs[ i ];
            auto bisector = tangentA + tangentB;
            bisector.turnPerpendicular();
            bisector.normalize();
            ribDirs[ i ] = bisector;
        }
    }

    // Now finally generate the offset points.
    const size_t lastRib = pos.size() - 1;
    for( size_t i = 0; i < pos.size(); i++ ) {
        const auto& ribCenter = pos[ i ];
        const auto& ribDir = ribDirs[ i ];

        // <X> This comes from the paper.
        double ribScaleFactor = 1.0;        
        if( i < lastRib || closedCurve ) {
            auto tangentPerp = ( i == lastRib && closedCurve ) ? tangentDirs.front() : tangentDirs[ i ];
            tangentPerp.turnPerpendicular();
            ribScaleFactor = std::min( maxRibFac,
                                       std::fabs( 1.0 / core::Vector2::dot( ribDir, tangentPerp ) ) );
        }
        storeLeft[ i ] = ribCenter - ribDir * 0.5 * leftWidths[ i ] * ribScaleFactor;
        storeRight[ i ] = ribCenter + ribDir * 0.5 * rightWidths[ i ] * ribScaleFactor;
    }
}

} // printCurves
