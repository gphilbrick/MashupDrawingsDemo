#include <utility/bspline2utility.h>
#include <utility/beziersfromspline.h>
#include <utility/linesegment.h>
#include <utility/mathutility.h>
#include <utility/curvesegment.h>

#include <map>

namespace core {

using Spline = BSpline2Utility::Spline;
/// A 'Spline' comprising a single Bezier curve.
using Bezier = Spline;
using UniqueSpline = BSpline2Utility::UniqueCurve;

using PairToCheck = CurveSegment::PairToCheck;
using PairsToCheck = CurveSegment::PairsToCheck;

namespace {

/// Perform De Casteljau splitting of the Bezier control polygon represented by 'controlPoints' and store the two resulting
/// control polygons in 'sectionA' and 'sectionB'.
void bisectControlPolygon( const std::vector< Vector2 >& controlPoints, std::vector< Vector2 >& sectionA, std::vector< Vector2 >& sectionB )
{
    // We are doing the simplified (t=0.5) division approach explained by Dr. Goldman.  Just build
    // the pyramid of values, where each pyramid is made of values which are averages of two values below.
    // The sectionA control points are the values on the left side of the pyramid and the sectionB control
    // points are the values on the right side of the pyramid;

    const size_t numControl = controlPoints.size();
    const size_t degree = numControl - 1;
    const size_t numLevels = degree + 1;
    Vector2** pyramidLevels = new Vector2*[ numLevels ];
    pyramidLevels[ 0 ] = new Vector2[ numControl ];
    for( size_t i = 0; i < numControl; i++ )
    {
        pyramidLevels[ 0 ][ i ] = controlPoints[ i ];
    }
    for( size_t level = 1; level < numLevels; level++ )
    {
        size_t levelWidth = degree + 1 - level;
        pyramidLevels[ level ] = new Vector2[ levelWidth ];
        for( size_t i = 0; i < levelWidth; i++ )
        {
            pyramidLevels[ level ][ i ] =
                ( pyramidLevels[ level - 1 ][ i ] + pyramidLevels[ level - 1 ][ i + 1 ] ) * 0.5;
        }
    }

    sectionA.resize( numControl );
    sectionB.resize( numControl );
    for( size_t i = 0; i < numControl; i++ )
    {
        sectionA[ i ] = pyramidLevels[ i ][ 0 ];
        sectionB[ i ] = pyramidLevels[ degree - i ][ i ];
    }

    for(size_t i = 0; i < numLevels; i++ )
    {
        delete[] pyramidLevels[ i ];
    }
    delete[] pyramidLevels;
}

/// If 'mapThisWaveToNextWave' has no entry for 'idx', bisect element 'idx' in 'thisWave' into its two children,
/// append these to 'nextWave', and add an entry in 'mapThisWaveToNextWave' mapping 'idx' to the penultimate element of
/// 'nextWave'. Return the index in 'nextWave' of 'idx''s first child (the value in 'mapThisWaveToNextWave' for key 'idx').
size_t addChildrenToNextWave( size_t idx,
    const CurveSegments& thisWave,
    CurveSegments& nextWave,
    std::map< size_t, size_t >& mapThisWaveToNextWave )
{
    const auto it = mapThisWaveToNextWave.find( idx );
    if( it == mapThisWaveToNextWave.end() ) {
        const CurveSegment& toSplit = thisWave[ idx ];

        nextWave.push_back( CurveSegment( toSplit.aOrB ) );
        nextWave.push_back( CurveSegment( toSplit.aOrB ) );
        CurveSegment& childA = *( nextWave.end() - 2 );
        CurveSegment& childB = *( nextWave.end() - 1 );

        bisectControlPolygon( toSplit.control, childA.control, childB.control );
        toSplit.tInterval.bisect( childA.tInterval, childB.tInterval );
        childA.bounds = BoundingBoxd( childA.control );
        childB.bounds = BoundingBoxd( childB.control );

        const size_t indexOfFirstChild = nextWave.size() - 2;
        mapThisWaveToNextWave[ idx ] = indexOfFirstChild;
        return indexOfFirstChild;
    } else {
        return it->second;
    }
}

/// Return whether both dimensions of 'box' are no larger than 'minBoxDim'.
bool boxCriticallySmall( const BoundingBoxd& box, double minBoxDim )
{
    return box.widthExclusive() <= minBoxDim && box.heightExclusive() <= minBoxDim;
}

/// Return whether 'toAdd' is far enough away from the intersections in 'existing'.
bool okToAddIntersection(
    const BoundingBoxd& toAdd,
    const CurveCurveIntersections& existing,
    double minDistBetweenIntersections )
{
    const auto intersectionCenter = toAdd.center();

    // Before we count this as an intersection, let's make sure it is not too close to
    // already found intersections.
    for( const auto& existingIntersection : existing )
    {
        if( toAdd.intersects( existingIntersection.hitBox ) ) {
            return false;
        }

        const auto existingCenter = existingIntersection.hitBox.center();
        if( ( existingCenter - intersectionCenter ).length() < minDistBetweenIntersections )
        {
            return false;
        }
    }
    return true;
}

/// Evaluate the t value for the nearest point to 'p' on non-null 'curve', which is a single Bezier curve.
/// Proceed until ascertaining that 'p' is no less than a and no greater than b from 'curve', where
/// b-a<='maxDistInterval'. If it becomes clear that 'p' is greater than 'shortCircuitDist' from 'curve',
/// return false--in this case, 'storeT' is left with no useful information.
bool nearestPointToBezier(
    const Vector2& p,
    const Bezier& bezier,
    double& storeT,
    double maxDistInterval,
    double shortCircuitDist )
{
    // Prevent getting stuck in an infinite loop.
    if( maxDistInterval <= 0 ) {
        return false;
    }

    // Part of 'curve'.
    struct Subcurve
    {
        // Declaring this kills the default copy operator; I want to avoid copies of
        // Subcurves to save time.
        Subcurve()
        {
        }

        BoundingBoxd box;
        std::vector< Vector2 > controlPoints;
        double tStart;
        double tEnd;
    };

    // Two buffers: one for using on the current round and one for storing the next round's contents; only
    // the first should be nonempty almost all the time (this is a way to avoid unnecessary copy operations).
    std::array< std::vector< Subcurve >, 2 > subcurvesBuffers;
    bool firstBufferActive = false;
    std::vector< Subcurve >* subcurves;
    std::vector< Subcurve >* subcurvesNext;
    const auto swapBuffers = [ & ]()
    {
        firstBufferActive = !firstBufferActive;
        subcurves = &subcurvesBuffers[ firstBufferActive ];
        subcurvesNext = &subcurvesBuffers[ !firstBufferActive ];
    };
    swapBuffers();

    // The initial subcurve is the whole curve.
    subcurves->resize( 1 );
    subcurves->at( 0 ).controlPoints = bezier.controlPoints();
    subcurves->at( 0 ).box = BoundingBoxd( subcurves->at( 0 ).controlPoints );
    subcurves->at( 0 ).tStart = 0.0;
    subcurves->at( 0 ).tEnd = 1.0;
    boost::optional< size_t > bestSubcurve = 0;

    bool searchShortCircuited = false;
    while( true ) {

        double closestPCanBeToCurve = std::numeric_limits< double >::max();

        // Find the best box, the one whose max distance to 'p' is smallest.
        double farthestDistToBestBox = std::numeric_limits< double >::max();
        bestSubcurve = boost::none;
        const size_t numSubcurves = subcurves->size();
        for( size_t i = 0; i < numSubcurves; i++ ) {
            const Subcurve& subcurve = subcurves->at( i );
            const BoundingBoxd& box = subcurve.box;
            const double farthestDist = mathUtility::distanceToFarthestPoint( p, box );
            const double nearestDist = mathUtility::distanceToNearestPoint( p, box );
            if( nearestDist < closestPCanBeToCurve ) {
                closestPCanBeToCurve = nearestDist;
            }
            if( farthestDist < farthestDistToBestBox ) {
                farthestDistToBestBox = farthestDist;
                bestSubcurve = i;
            }
        }

        // Do we not care about iterating further because we already know that 'p' is
        // too far away from 'curve' for its exact distance from 'curve' to matter?
        if( closestPCanBeToCurve > shortCircuitDist ) {
            searchShortCircuited = true;
            break;
        }

        // Is the nearest box small enough for iteration to terminate?
        const double nearestDistToBestBox =
            mathUtility::distanceToNearestPoint( p, subcurves->at( bestSubcurve.value() ).box );
        if( farthestDistToBestBox - nearestDistToBestBox <= maxDistInterval ) {
            break;
        }

        // Get ready for the next round by splitting up all the current subcurves. Populate
        // 'subCurvesNext'.
        subcurvesNext->resize( 0 );
        for( size_t i = 0; i < numSubcurves; i++ ) {
            const Subcurve& subcurve = subcurves->at( i );

            // Cut out this subcurve if the nearest it could come to 'p' is farther
            // than the maximum distance some other subcurve can be to 'p'.
            const double nearestDist = mathUtility::distanceToNearestPoint( p, subcurve.box );
            if( nearestDist < farthestDistToBestBox ) {
                subcurvesNext->resize( subcurvesNext->size() + 2 );

                Subcurve& first = ( *subcurvesNext ).end()[ -2 ];
                Subcurve& second = ( *subcurvesNext ).end()[ -1 ];

                const double tMid = ( subcurve.tStart + subcurve.tEnd ) / 2.0;
                first.tStart = subcurve.tStart;
                first.tEnd = tMid;
                second.tStart = tMid;
                second.tEnd = subcurve.tEnd;
                // Control points
                bisectControlPolygon( subcurve.controlPoints, first.controlPoints, second.controlPoints );
                // Bounding boxes.
                first.box = BoundingBoxd( first.controlPoints );
                second.box = BoundingBoxd( second.controlPoints );
            }
        }
        swapBuffers();
    }

    if( searchShortCircuited ) {
        return false;
    } else {
        // The best t value is the midpoint t value of the nearest box to 'p'.
        const Subcurve& best = subcurves->at( bestSubcurve.value() );
        storeT = ( best.tStart + best.tEnd ) / 2.0;
        return true;
    }
}

/// Calculate the intersections between 'a' and 'b', each of which comprises a single Bezier curve, and
/// add the intersections to 'intersectionsToAddTo', making sure not to duplicate the intersections already there.
void bezierIntersections(
    const Bezier& a,
    const Bezier& b,
    CurveCurveIntersections& intersections,
    const IntersectionParameters& params )
{
    // Deliberately not clearing 'intersections'.

    std::array< CurveSegments, 2 > segmentsBuffers;
    std::array< PairsToCheck, 2 > pairsToCheckBuffers;
    CurveSegments* segments = 0;
    CurveSegments* segmentsNext = 0;
    PairsToCheck* pairsToCheck = 0;
    PairsToCheck* pairsToCheckNext = 0;
    bool readIndex = false;

    // Maps from current index in 'segments' to the first of the two child indices in the _next_ 'segments'.
    std::map< size_t, size_t > currentToFirstChildInNext;

    // Set up the initial read buffers: two curve segments and a single pair.
    {
        segmentsBuffers[ 1 ].resize( 2 );
        CurveSegment::wholeCurveToSegment( a, true, segmentsBuffers[ 1 ][ 0 ] );
        CurveSegment::wholeCurveToSegment( b, false, segmentsBuffers[ 1 ][ 1 ] );
        pairsToCheckBuffers[ 1 ].insert( PairToCheck{ 0, 1 } );
    }

    while( true ) {
        readIndex = !readIndex;
        segments = &segmentsBuffers[ readIndex ];
        segmentsNext = &segmentsBuffers[ !readIndex ];
        segmentsNext->clear();
        pairsToCheck = &pairsToCheckBuffers[ readIndex ];
        pairsToCheckNext = &pairsToCheckBuffers[ !readIndex ];
        pairsToCheckNext->clear();
        currentToFirstChildInNext.clear();

        if( pairsToCheck->size() == 0 ) {
            break;
        }

        // Check all the scheduled pairs of curve segments--each pair should include a segment from 'a' and one from 'b'.
        // Whenever there is an intersection, make a note that in the next wave we need to check the two children of
        // the 'a' segment against the two children of the 'b' segment.

        for( const auto& pair : *pairsToCheck ) {
            const CurveSegment& seg1 = segments->at( pair.first );
            const CurveSegment& seg2 = segments->at( pair.second );

            if( seg1.bounds.intersects( seg2.bounds ) ) {

                // Is this segment-segment intersection small enough for termination?
                BoundingBoxd combinedBox = seg1.bounds;
                combinedBox.growToContain( seg2.bounds );
                if( boxCriticallySmall( combinedBox, params.minBoxDim ) ) {
                    const bool okToAdd = okToAddIntersection( combinedBox, intersections, params.minDistBetweenIntersections );
                    if( okToAdd )
                    {
                        CurveCurveIntersection intersection;
                        intersection.hitBox = combinedBox;
                        intersection.tIntervalA = seg1.aOrB ? seg1.tInterval : seg2.tInterval;
                        intersection.tIntervalB = seg1.aOrB ? seg2.tInterval : seg1.tInterval;
                        intersections.push_back( intersection );
                    }
                } else {
                    // Make sure both segments have their children in the next wave, if this hasn't already been done.
                    const size_t indexOfSeg1Child = addChildrenToNextWave( pair.first,
                        *segments,
                        *segmentsNext,
                        currentToFirstChildInNext );
                    const size_t indexOfSeg2Child = addChildrenToNextWave( pair.second,
                        *segments,
                        *segmentsNext,
                        currentToFirstChildInNext );

                    // Now schedule intersection tests between the two children of 'seg1' and the two children of 'seg2' in the next
                    // wave.
                    for( size_t i = 0; i < 2; i++ ) {
                        for( size_t j = 0; j < 2; j++ ) {
                            pairsToCheckNext->insert( std::minmax( indexOfSeg1Child + i, indexOfSeg2Child + j ) );
                        }
                    }
                }
            }
        }
    }
}

} // unnamed

void BSpline2Utility::intersections(
    const Spline& a,
    const Spline& b,
    CurveCurveIntersections& intersections,
    const IntersectionParameters& params )
{
    intersections.clear();

    if( !a.boundingBox().intersects( b.boundingBox() ) ) {
        return;
    }

    // Collate the intersections between each pair of component Bezier curves.
    const BeziersFromSpline beziersFromA( a );
    const auto& aCurveTStarts = beziersFromA.tStarts();
    const auto& aCurves = beziersFromA.beziers();
    const BeziersFromSpline beziersFromB( b );
    const auto& bCurveTStarts = beziersFromB.tStarts();
    const auto& bCurves = beziersFromB.beziers();

    // Store the intersections between every pair of curves.
    const size_t numACurves = aCurves.size();
    const size_t numBCurves = bCurves.size();
    const BoundingIntervald defaultTInterval( 0, 1 );
    for( size_t i = 0; i < numACurves; i++ ) {
        const auto& aCurve = *aCurves[ i ];
        const BoundingIntervald aTInterval(
            aCurveTStarts[ i ],
            i == numACurves - 1 ? 1.0 : aCurveTStarts[ i + 1 ] );
        for( size_t j = 0; j < numBCurves; j++ ) {
            const auto& bCurve = *bCurves[ j ];
            const BoundingIntervald bTInterval(
                bCurveTStarts[ j ],
                j == numBCurves - 1 ? 1.0 : bCurveTStarts[ j + 1 ] );

            const size_t numIntersectionsBefore = intersections.size();
            bezierIntersections( aCurve, bCurve, intersections, params );
            // The intersections we've just added need to be adjusted so that their T intervals are relative to the
            // splines they come from, not the Bezier curves 'aCurve' and 'bCurve'.
            const size_t numToAdjust = intersections.size() - numIntersectionsBefore;
            for( auto it = intersections.end() - numToAdjust; it != intersections.end(); it++ ) {
                CurveCurveIntersection& toFix = *it;
                toFix.tIntervalA.remap( defaultTInterval, aTInterval );
                toFix.tIntervalB.remap( defaultTInterval, bTInterval );
            }
        }
    }
}

void BSpline2Utility::selfIntersections(
    const Spline& spline,
    CurveCurveIntersections& intersections,
    const IntersectionParameters& params )
{
    intersections.clear();

    // Prevent size_t wraparound risk in 'totalAngleChange'.
    if( spline.degree() < 1 ) {
        return;
    }

    // The algorithm here comes from Dieter Lasser's "Calculating the self-intersections of Bézier curves," with personal tweaks.
    //
    // Genus 1: A single subcurve (a small piece of a Bezier curve produced by De Casteljau) intersects itself--a cusp. The angle
    // test--does the total abs-value angle change based on the control polygon exceed pi--indicates when a single subcurve _might_
    // self-intersect. I choose not to _directly_ produce an intersection from the genus-2 case but to just keep subdividing until
    // a genus-3 intersection occurs instead or the bounding box of the genus-1 curve gets so small that we just discard it.
    //
    // Genus 2: Same idea as genus 1, except for a pair of T-adjacent subcurves which share a control point. Again, just keep
    // subdividing until bounding boxes get too small or a genus-3 intersection occurs.
    //
    // Genus 3: A pair of subcurves which do not share a control point. Use the straightforward subdivision-with-bounding-boxes approach on
    // these pairs.

    std::array< CurveSegments, 2 > segmentsBuffers;
    CurveSegments* segments = 0;
    CurveSegments* segmentsNext = 0;

    using IndicesToCheck = std::set< size_t >;
    std::array< IndicesToCheck, 2 > genus1Buffers;
    IndicesToCheck* genus1 = 0;
    IndicesToCheck* genus1Next = 0;

    using PairsToCheck = CurveSegment::PairsToCheck;
    using PairsToCheckBuffers = std::array< PairsToCheck, 2 >;
    PairsToCheckBuffers genus2Buffers;
    PairsToCheck* genus2 = 0;
    PairsToCheck* genus2Next = 0;
    PairsToCheckBuffers genus3Buffers;
    PairsToCheck* genus3 = 0;
    PairsToCheck* genus3Next = 0;

    // Swap-buffer mechanism.
    bool readIndex = false;

    // Maps from current index in 'segments' to the first of the two child indices in the _next_ 'segments'.
    std::map< size_t, size_t > currentToFirstChildInNext;

    // Set up initial wave
    {
        // Lasser briefly covers the spline case (multiple connected Bezier curves) by saying to find all the component Bezier curves
        // and perform the self-intersect algorithm on each individually and then to perform typical subdivision-based curve-curve intersect
        // tests on pairs of component Beziers. This doesn't make sense, since some of those pairs will share control points and therefore produce
        // bogus intersections where they connect.
        //
        // I'm instead treating the whole initial spline as if it were a single Bezier, representing adjacent Bezier curves as genus-2 pairs. Whether
        // this is valid comes down to whether the genus-2 angle-change test is still valid--and I don't see why it wouldn't be, since when you normally
        // perform this test on two Bezier subcurves, it is essentially the same as performing it on two Bezier curves adjacent within a spline.

        const BeziersFromSpline beziersFromSpline( spline );
        const auto& tStarts = beziersFromSpline.tStarts();
        const auto& curves = beziersFromSpline.beziers();

        // Filter out zero-length curves.
        size_t numValidCurves = 0;
        for( size_t i = 0; i < curves.size(); i++ ) {
            const BoundingBoxd bounds = curves[ i ]->boundingBox();

            if( bounds.widthExclusive() != 0 || bounds.heightExclusive() != 0 ) {
                CurveSegment subcurve;
                CurveSegment::wholeCurveToSegment( *curves[ i ], true, subcurve );
                subcurve.tInterval = { tStarts[ i ], i == curves.size() - 1 ? 1.0 : tStarts[ i + 1 ] };
                segmentsBuffers[ 1 ].push_back( subcurve );
                numValidCurves++;
            }
        }

        for( size_t i = 0; i < numValidCurves; i++ ) {
            genus1Buffers[ 1 ].insert( i );
            if( i < numValidCurves - 1 ) {
                genus2Buffers[ 1 ].insert( { i, i + 1 } );
            }
            for( size_t j = i + 2; j < numValidCurves; j++ ) {
                genus3Buffers[ 1 ].insert( { i, j } );
            }
        }
    }

    // Compute the total angle change across a control polygon.
    const auto totalAngleChange = []( const std::vector< Vector2 >& control ) -> double
    {
        if( control.size() > 0 ) {
            double sum = 0;
            for( size_t i = 1; i < control.size() - 1; i++ ) {
                sum += mathUtility::absAngleChange(
                    control[ i - 1 ], control[ i ], control[ i + 1 ] );
            }
            return sum;
        } else {
            return 0.0; // Should never get here.
        }
    };

    while( true ) {
        readIndex = !readIndex;
        segments = &segmentsBuffers[ readIndex ];
        segmentsNext = &segmentsBuffers[ !readIndex ];
        segmentsNext->clear();
        genus1 = &genus1Buffers[ readIndex ];
        genus1Next = &genus1Buffers[ !readIndex ];
        genus1Next->clear();
        genus2 = &genus2Buffers[ readIndex ];
        genus2Next = &genus2Buffers[ !readIndex ];
        genus2Next->clear();
        genus3 = &genus3Buffers[ readIndex ];
        genus3Next = &genus3Buffers[ !readIndex ];
        genus3Next->clear();

        currentToFirstChildInNext.clear();

        if( genus1->size() == 0 && genus2->size() == 0 && genus3->size() == 0 ) {
            break;
        }

        // Genus 1: Look at individual subcurves. If one passes the angle test, it might self-intersect, meaning we split it into
        // two curves: two more genus-1s and one genus-2 pair.
        for( const auto& idx : *genus1 ) {
            const CurveSegment& seg = segments->at( idx );
            if( !boxCriticallySmall( seg.bounds, params.minBoxDim ) ) {
                const double angleSum = totalAngleChange( seg.control );
                if( angleSum > boost::math::constants::pi< double >() ) {
                    const size_t firstChild = addChildrenToNextWave( idx,
                        *segments,
                        *segmentsNext,
                        currentToFirstChildInNext );
                    genus1Next->insert( firstChild );
                    genus1Next->insert( firstChild + 1 );
                    genus2Next->insert( { firstChild, firstChild + 1 } );
                }
            }
        }

        // Genus 2: Look at each pair of control point-sharing curves. If, as a unit, the pair passes the angle test,
        // then split both curves, producing four subcurves in the next wave: four genus-1 cases, 3 genus-2 pairs, and 3 genus-3 pairs.
        // Lasser's paper says to produce only one genus-2 pair and 3 genus-3 pairs, but I assume this is a mistake.
        for( const auto& pair : *genus2 ) {
            const CurveSegment& seg1 = segments->at( pair.first );
            const CurveSegment& seg2 = segments->at( pair.second );

            BoundingBoxd combined = seg1.bounds;
            combined.growToContain( seg2.bounds );
            if( !boxCriticallySmall( combined, params.minBoxDim ) ) {
                const double angleSum = totalAngleChange( seg1.control ) + totalAngleChange( seg2.control );
                if( angleSum > boost::math::constants::pi< double >() ) {
                    const size_t seg1FirstChild = addChildrenToNextWave( pair.first,
                        *segments,
                        *segmentsNext,
                        currentToFirstChildInNext );
                    const size_t seg2FirstChild = addChildrenToNextWave( pair.second,
                        *segments,
                        *segmentsNext,
                        currentToFirstChildInNext );

                    // Genus-1s in next wave.
                    for( size_t i = 0; i < 2; i++ ) {
                        genus1Next->insert( seg1FirstChild + i );
                        genus1Next->insert( seg2FirstChild + i );
                    }

                    // Genus-2s in next wave.
                    genus2Next->insert( { seg1FirstChild, seg1FirstChild + 1 } );
                    genus2Next->insert( { seg2FirstChild, seg2FirstChild + 1 } );
                    genus2Next->insert( std::minmax( seg1FirstChild + 1, seg2FirstChild ) );

                    // Genus-3s in next wave
                    genus3Next->insert( std::minmax( seg1FirstChild, seg2FirstChild ) );
                    genus3Next->insert( std::minmax( seg1FirstChild + 1, seg2FirstChild + 1 ) );
                    genus3Next->insert( std::minmax( seg1FirstChild, seg2FirstChild + 1 ) );
                }
            }
        }

        // Genus 3: For each pair of non-connected subcurves, find out if the bounding boxes intersect. If so, subdivide
        // and produce four new genus-3 pairs.
        for( const auto& pair : *genus3 ) {
            const CurveSegment& seg1 = segments->at( pair.first );
            const CurveSegment& seg2 = segments->at( pair.second );

            if( seg1.bounds.intersects( seg2.bounds ) ) {
                // Is this small enough to flag as an intersection?
                BoundingBoxd combined = seg1.bounds;
                combined.growToContain( seg2.bounds );
                if( boxCriticallySmall( combined, params.minBoxDim ) ) {
                    const bool okToAdd = okToAddIntersection( combined, intersections, params.minDistBetweenIntersections );
                    if( okToAdd )
                    {
                        CurveCurveIntersection intersection;
                        intersection.hitBox = combined;
                        if( seg1.tInterval.min() < seg2.tInterval.min() ) {
                            intersection.tIntervalA = seg1.tInterval;
                            intersection.tIntervalB = seg2.tInterval;
                        } else {
                            intersection.tIntervalA = seg2.tInterval;
                            intersection.tIntervalB = seg1.tInterval;
                        }
                        intersections.push_back( intersection );
                    }
                } else {
                    // Subdivide further.
                    const size_t seg1FirstChild = addChildrenToNextWave( pair.first,
                        *segments,
                        *segmentsNext,
                        currentToFirstChildInNext );
                    const size_t seg2FirstChild = addChildrenToNextWave( pair.second,
                        *segments,
                        *segmentsNext,
                        currentToFirstChildInNext );

                    for( size_t i = 0; i < 2; i++ ) {
                        for( size_t j = 0; j < 2; j++ ) {
                            genus3Next->insert( std::minmax( seg1FirstChild + i, seg2FirstChild + j ) );
                        }
                    }
                }
            }
        } // genus-3
    }
}

double BSpline2Utility::nearestPoint( const Vector2& p, const Spline& spline, double maxDistInterval )
{
    // Find nearest points on each of the constituent curves.
    const BeziersFromSpline beziersFromSpline( spline );
    const auto& tStarts = beziersFromSpline.tStarts();
    const auto& curves = beziersFromSpline.beziers();

    double closestDistToCurve = std::numeric_limits< double >::max();
    double bestSplineT = 0;

    for( size_t i = 0; i < curves.size(); i++ ) {
        const auto& curve = *curves[ i ];
        double curveT = 0.0;
        if( nearestPointToBezier( p, curve, curveT, maxDistInterval, closestDistToCurve ) ) {
            const Vector2 pointOnCurve = curve.position( curveT );
            const double distToCurve = ( pointOnCurve - p ).length();
            if( distToCurve < closestDistToCurve ) {
                closestDistToCurve = distToCurve;
                // Convert from curve t to spline t.
                const double& splineCurveStart = tStarts[ i ];
                const double& splineCurveEnd = i == curves.size() - 1 ? 1.0 : tStarts[ i + 1 ];
                bestSplineT = splineCurveStart + curveT * ( splineCurveEnd - splineCurveStart );
            }
        }
    }
    return bestSplineT;
}

void BSpline2Utility::lineSegmentIntersections(
    const Spline& a,
    const LineSegment& b,
    CurveCurveIntersections& intersections,
    const IntersectionParameters& params )
{
    lineSegmentIntersections( a, b.a, b.b, intersections, params );
}

void BSpline2Utility::lineSegmentIntersections(
    const Spline& a,
    const Vector2& bStart,
    const Vector2& bEnd,
    CurveCurveIntersections& storeIntersections,
    const IntersectionParameters& params )
{
    const std::vector< Vector2 > controlPoints { bStart, bEnd };
    const Spline b( 1, controlPoints );
    intersections( a, b, storeIntersections, params );
}

UniqueSpline BSpline2Utility::addSplines( const std::vector< const Spline* >& splines )
{
    if( splines.size() == 0 ) {
        return nullptr;
    }

    int maxDegree = 0;
    std::vector< UniqueCurve > copies( splines.size() );
    for( size_t i = 0; i < copies.size(); i++ ) {
        copies[ i ] = std::make_unique< Spline >( *splines[ i ] );
        if( splines[ i ]->degree() > maxDegree ) {
            maxDegree = splines[ i ]->degree();
        }
    }

    // Make sure all have the same degree.
    for( auto& copy : copies ) {
        if( copy->degree() < maxDegree ) {
            copy->degreeElevate( maxDegree );
        }
    }

    // Make them all have the same knots.
    for( size_t i = 0; i < copies.size(); i++ ) {
        for( size_t j = i + 1; j < copies.size(); j++ ) {
            unionKnotVectors( *copies[ i ], *copies[ j ] );
        }
    }

    std::vector< Vector2 > sumControl( copies[ 0 ]->controlPoints().size() );
    for( size_t i = 0; i < sumControl.size(); i++ ) {
        for( auto& term : copies ) {
            sumControl[ i ] += term->controlPoints()[ i ];
        }
    }
    return Spline::createFromControlPointsAndKnots( copies[ 0 ]->degree(), sumControl, copies[ 0 ]->internalKnots() );
}

UniqueSpline BSpline2Utility::interpolateBetweenSplines( const Spline& a, const Spline& b, double t )
{
    auto aCopy = std::make_unique< Spline >( a );
    const auto bCopy = std::make_unique< Spline >( b );

    if( aCopy->degree() < bCopy->degree() ) {
        aCopy->degreeElevate( bCopy->degree() );
    } else if ( bCopy->degree() < aCopy->degree() ) {
        bCopy->degreeElevate( aCopy->degree() );
    }
    unionKnotVectors( *aCopy, *bCopy );

    // At this point, both should have the same number of control points, which means that interpolation is just a matter of interpolating
    // control points.
    const auto& aControl = aCopy->controlPoints();
    const auto& bControl = bCopy->controlPoints();
    Spline::Control control( aControl.size() );
    for( size_t i = 0; i < control.size(); i++ ) {
        control[ i ] = aControl[ i ] + ( bControl[ i ] - aControl[ i ] ) * t;
    }
    return Spline::createFromControlPointsAndKnots( aCopy->degree(), control, aCopy->internalKnots() );
}

void BSpline2Utility::unionKnotVectors( Spline& a, Spline& b )
{
    struct ToAdd
    {
        double knotValue;
        int numToAddToS;
        int numAlreadyInS;
        int firstKnotIdxInS; // in Sederberg knot vector.
    };

    // Find out what knots 's' needs from 'intermediateKnots'.
    auto createKnotOrder = []( const Spline& s, const std::vector< double >& intermediateKnots )
    {
        std::vector< ToAdd > knotsToAdd;
        const std::vector< double > sKnots = s.internalKnots();

        // Compare the knots of 's' with 'intermediateKnots' to see what 's' lacks.
        size_t i = 0;
        size_t i_s = 0;
        while( i < intermediateKnots.size() ) {
            const double knotToAdd = intermediateKnots[ i ];
            int multiplicityInInput = 0;
            do {
                multiplicityInInput++;
                i++;
            } while( i < intermediateKnots.size()
                && mathUtility::closeEnough( intermediateKnots[ i ], knotToAdd ) );

            // Does this same knot occur the same number of times in
            int multiplicityInS = 0;
            while( i_s < sKnots.size()
                && sKnots[ i_s ] < knotToAdd
                && !mathUtility::closeEnough( sKnots[ i_s ], knotToAdd ) ) {
                i_s++;
            }
            int insertLoc = static_cast< int >( i_s );
            while( i_s < sKnots.size()
                && mathUtility::closeEnough( sKnots[ i_s ], knotToAdd ) ) {
                multiplicityInS++;
                i_s++;
            }

            ToAdd toAdd;
            toAdd.knotValue = knotToAdd;
            toAdd.numAlreadyInS = multiplicityInS;
            toAdd.numToAddToS = multiplicityInInput - multiplicityInS;
            toAdd.firstKnotIdxInS = insertLoc + s.degree();
            knotsToAdd.push_back( toAdd );
        }

        return knotsToAdd;
    };

    // Supplement 's' with whatever it was missing.
    auto fillKnotOrder = []( Spline& s, const std::vector< ToAdd >& order )
    {
        // Handle the knots by decreasing insertion index.
        if( order.size() ) {
            // Supplement 's' with the missing knots.
            std::vector< double > knots = s.fullKnots();
            std::vector< Vector2 > control = s.controlPoints();

            for( size_t i = order.size() - 1; i < order.size(); i-- ) {
                const ToAdd& toAdd = order[ i ];
                for( int j = 0; j < toAdd.numToAddToS; j++ ) {
                    insertKnot( toAdd.firstKnotIdxInS, toAdd.knotValue, toAdd.numAlreadyInS + j, s.degree(), knots, control );
                }
            }
            const std::vector< double > intermediateKnotsOnly( knots.begin() + s.degree(), knots.end() - s.degree() );
            s.buildFromControlPointsAndKnots( s.degree(), control, intermediateKnotsOnly );
        }

    };

    auto orderForA = createKnotOrder( a, b.internalKnots() );
    auto orderForB = createKnotOrder( b, a.internalKnots() );
    fillKnotOrder( a, orderForA );
    fillKnotOrder( b, orderForB );
}

UniqueSpline BSpline2Utility::stitchC0Spline(
    const std::vector< const Spline* >& parts,
    const size_t lengthPrecision,
    bool closedShape,
    std::vector< double >* storePartEndT )
{
    std::vector< double > tWeights( parts.size() );
    for( size_t i = 0; i < parts.size(); i++ ) {
        tWeights[ i ] = parts[ i ]->cachedLength( lengthPrecision );
    }
    return stitchC0Spline( parts, tWeights, closedShape, storePartEndT );
}

UniqueSpline BSpline2Utility::stitchC0Spline(
    const std::vector< const Spline* >& partsDegreeHetero,
    const std::vector< double >& tWeights,
    bool closedShape,
    std::vector< double >* storePartEndT ) {

    if( partsDegreeHetero.size() == 1 ) {
        const auto& toCopy = *partsDegreeHetero.front();
        if( closedShape ) {
            auto control = toCopy.controlPoints();
            control.back() = control.front();
            const auto intermediateKnots = toCopy.internalKnots();
            return Spline::spline( toCopy.degree(), control, intermediateKnots );
        } else {
            return std::make_unique< Spline >( toCopy );
        }
    }

    // What T interval will each part get?
    const auto tEnds = tEndValues( tWeights );
    if( storePartEndT ) {
        *storePartEndT = tEnds;
    }

    // Make sure all have same degree
    int maxDegree = 0;
    std::vector< std::unique_ptr< Spline > > parts( partsDegreeHetero.size() );
    for( size_t i = 0; i < parts.size(); i++ ) {
        parts[ i ] = std::make_unique< Spline >( *partsDegreeHetero[ i ] );
        if( parts[ i ]->degree() > maxDegree ) {
            maxDegree = parts[ i ]->degree();
        }
    }
    for( size_t i = 0; i < parts.size(); i++ ) {
        if( parts[ i ]->degree() < maxDegree ) {
            parts[ i ]->degreeElevate( maxDegree );
        }
    }

    Spline::Control control;
    std::vector< double > internalKnots;
    for( size_t i = 0; i < parts.size(); i++ ) {
        const double tEnd = tEnds[ i ];
        const double tStart = i == 0 ? 0.0 : tEnds[ i - 1 ];

        const auto& partControl = parts[ i ]->controlPoints();

        // Append the next part's control points.
        auto begin = ( i == 0 ? partControl.begin() : partControl.begin() + 1 );
        control.insert( control.end(), begin, partControl.end() );

        // Multiple knots to form the C0 juncture.
        if( i != 0 ) {
            for( int j = 0; j < maxDegree; j++ ) {
                internalKnots.push_back( tStart );
            }
        }

        const std::vector< double > partInternalKnots = parts[ i ]->internalKnots();
        for( size_t k = 0; k < partInternalKnots.size(); k++ ) {
            internalKnots.push_back( tStart + partInternalKnots[ k ] * ( tEnd - tStart ) );
        }
    }

    if( closedShape ) {
        control.back() = control.front();
    }
    return Spline::createFromControlPointsAndKnots( maxDegree, control, internalKnots );
}

std::vector< double > BSpline2Utility::tEndValues( const std::vector< double >& componentWeights )
{
    std::vector< double > tEnds( componentWeights.size() );
    {
        double weightSum = 0;
        for( size_t i = 0; i < componentWeights.size(); i++ ) {
            weightSum += componentWeights[ i ];
            tEnds[ i ] = weightSum;
        }
        for( size_t i = 0; i < componentWeights.size(); i++ ) {
            tEnds[ i ] /= weightSum;
        }
    }
    return tEnds;
}

void BSpline2Utility::insertKnot(
    int i, double knotValue, int numExistingCopies, int degree, std::vector< double >& knots, Spline::Control& control )
{
    // New control point to place right before the control point currently indexed by 'i'.
    Vector2 latestControlPoint = i == 0
        ? control[ 0 ]
        : interpFromControlPoints( i, knotValue, degree, control, knots );

    // Duplicating the knot at 'i' invalidates _degree - 2 - ( numExistingCopies - 1 ) existing control points.
    const int numPrevToUpdate = degree - 2 - ( numExistingCopies - 1 );
    if( numPrevToUpdate > 0 ) {
        std::vector< Vector2 > updatedPrevPoints( numPrevToUpdate );
        for( int j = 0; j < numPrevToUpdate; j++ ) {
            updatedPrevPoints[ j ] = interpFromControlPoints(
                i - degree + 1 + numExistingCopies + j, knotValue, degree, control, knots );
        }
        for( int j = 0; j < numPrevToUpdate; j++ ) {
            control[ i - degree + 1 + numExistingCopies + j ] = updatedPrevPoints[ j ];
        }
    }
    control.insert( control.begin() + i, latestControlPoint );
    knots.insert( knots.begin() + i, knotValue );
}

Vector2 BSpline2Utility::interpFromControlPoints(
    int i, double knot, int degree, const std::vector< Vector2 >& control, const std::vector< double >& knots )
{
    const double knotA = knots[ i - 1 ];
    const double knotB = knots[ i + degree - 1 ];
    const double u = ( knot - knotA ) / ( knotB - knotA );
    const Vector2& a = control[ i - 1 ];
    const Vector2& b = control[ i ];
    const Vector2 lerp = a + ( b - a ) * u;
    return lerp;
}

} // core
