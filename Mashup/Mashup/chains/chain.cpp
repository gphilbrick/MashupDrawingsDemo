#include <chains/chain.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/model/curveback.h>
#include <Core/model/rgbback.h>
#include <Core/model/stroketools.h>

#include <Core/utility/mathutility.h>

namespace mashup {
namespace chains {

using Pos = core::model::Pos;
using UniqueStroke = core::model::UniqueStroke;

std::unique_ptr< Chain > Chain::checkAndCombineHalves( const Chain& a, const Chain& b )
{
    // Sweeping for chain-building bugs here...

    if( a.substrokes.size() == 0 || b.substrokes.size() == 0 ) {
        THROW_UNEXPECTED;
    }

    // Look at the fronts of both the chains, and build 'midSS', the 'Substroke'
    // that connects the two chains.
    Substroke midSS;
    {
        const auto& aFront = a.substrokes.front();
        const auto& bFront = b.substrokes.front();

        // Due to trimming, these will not necessarily be reverse() copies of each other.
        if( aFront.stroke != bFront.stroke || aFront.tIncreasing() == bFront.tIncreasing() ) {
            THROW_UNEXPECTED;
        }
        const auto* const stroke = aFront.stroke;

        if( aFront.tIncreasing() ) {
            if( bFront.t[ 1 ] > aFront.t[ 1 ] ) {
                THROW_UNEXPECTED;
            }
        } else {
            if( bFront.t[ 1 ] < aFront.t[ 1 ] ) {
                THROW_UNEXPECTED;
            }
        }
        midSS = Substroke( *stroke, bFront.t[ 1 ], aFront.t[ 1 ] );
    }

    if( a.closed && b.closed ) {
        if( a.substrokes.size() == 1 && b.substrokes.size() == 1 ) {
            return a.clone();
        } else {
            THROW_UNEXPECTED;
        }
    } else if( a.closed ) {
        if( a.substrokes.size() != a.joints.size() ) {
            THROW_UNEXPECTED;
        }
        return a.clone();
    } else if( b.closed ) {
        if( b.substrokes.size() != b.joints.size() ) {
            THROW_UNEXPECTED;
        }
        return b.clone();
    }

    if( a.substrokes.size() != a.joints.size() + 1 ) {
        THROW_UNEXPECTED;
    }
    if( b.substrokes.size() != b.joints.size() + 1 ) {
        THROW_UNEXPECTED;
    }

    auto ret = std::make_unique< Chain >();

    // Put all but first substroke of 'b' into 'ret', in reverse
    const auto numB_SS = b.substrokes.size();
    for( size_t i = 0; i < numB_SS - 1; i++ ) {
        ret->substrokes.push_back( b.substrokes[ numB_SS - 1 - i ].reverse() );
        const auto& joint = b.joints[ numB_SS - 2 - i ];
        ret->joints.push_back( joint ? joint->reverse() : nullptr );
    }

    ret->substrokes.push_back( midSS );

    // Put all of 'a' except first substroke into 'ret', in order
    for( size_t i = 1; i < a.substrokes.size(); i++ ) {
        ret->substrokes.push_back( a.substrokes[ i ] );
    }
    for( const auto& j : a.joints ) {
        ret->joints.push_back( j ? j->clone() : nullptr );
    }

    return ret;
}

bool Chain::hasTail( bool startOrEnd ) const
{
    if( closed ) {
        return false;
    }

    if( startOrEnd ) {
        return !substrokes.front().includesStrokeEndpoint( false );
    } else {
        return !substrokes.back().includesStrokeEndpoint( true );
    }
}

std::unique_ptr< Chain > Chain::clone() const
{
    auto ret = std::make_unique< Chain >();

    ret->closed = closed;
    ret->substrokes = substrokes;

    ret->joints.resize( joints.size() );
    for( size_t i = 0; i < joints.size(); i++ ) {
        ret->joints[ i ] = joints[ i ]
                ? joints[ i ]->clone()
                : nullptr;
    }

    return ret;
}

std::unique_ptr< Chain > Chain::simplified() const
{
    if( substrokes.size() < 2 ) {
        return clone();
    }

    std::unique_ptr< Chain > retHandle = std::make_unique< Chain >();
    auto& ret = *retHandle;
    ret.closed = closed;

    boost::optional< Substroke > toAdd;
    for( size_t i = 0; i < substrokes.size(); i++ ) {
        const auto& ss = substrokes[ i ];
        if( toAdd ) {
            if( joints[ i - 1 ] ) {
                // 'i-1' and 'i' cannot be collapsed
                ret.substrokes.push_back( *toAdd );
                ret.joints.push_back( joints[ i - 1 ]->clone() );
                toAdd = ss;
            } else {
                // 'i-1' and 'i' can be collapsed
                toAdd->t[ 1 ] = ss.t[ 1 ];
            }
        } else {
            toAdd = ss;
        }
    }

    if( toAdd ) {
        if( ret.closed ) {
            const auto& lastJoint = joints.back();

            if( lastJoint ) {
                // 'last' and 'first' do not represent a single sub-stroke
                ret.substrokes.push_back( *toAdd );
                ret.joints.push_back( lastJoint->clone() );
            } else {
                // 'last' and 'first' can be represented as a single sub-stroke
                if( ret.substrokes.size() == 1 ) {
                    // This has to mean that our chain is just a single looping 'Stroke'.
                    ret.substrokes.front().t = { 0., 1. };
                } else {
                    // This means that our chain involves multiple strokes.
                    ret.substrokes.front().t[ 0 ] = toAdd->t[ 0 ];
                }
            }
        } else {
            ret.substrokes.push_back( *toAdd );
        }
    }

    return retHandle;
}

UniqueStroke Chain::stroke() const
{
    std::vector< UniqueStroke > toStitch;
    const auto numSubs = substrokes.size();
    if( numSubs == 0 ) {
        THROW_UNEXPECTED;
    }
    for( size_t i = 0; i < numSubs; i++ ) {
        const auto& ssA = substrokes[ i ];
        toStitch.push_back( ssA.asStroke() );
        if( closed || i < numSubs - 1 ) {
            if( joints[ i ] ) {
                toStitch.push_back( joints[ i ]->clone() );
            }
        }
    }
    return core::model::stitchC0Strokes( toStitch, closed );
}

bool Chain::hasBadJoint( double maxEndpointMismatch, double* storeBadDist ) const
{
    if( substrokes.size() < 1 ) {
        return false;
    }

    const auto tooFar = [ & ]( const Pos& a, const Pos& b )
    {
        const auto dist = ( b - a ).length();
        if( dist > maxEndpointMismatch ) {
            if( storeBadDist ) {
                *storeBadDist = dist;
            }
            return true;
        } else {
            return false;
        }
    };

    const size_t numJoints = closed ? substrokes.size() : substrokes.size() - 1;
    for( size_t j = 0; j < numJoints; j++ ) {
        if( !joints[ j ] ) {
            continue;
        }

        const auto& jCurve = joints[ j ]->curve();

        const auto& ssA = substrokes[ j ];
        if( tooFar( ssA.endpoint( true ), jCurve.startPosition() ) ) {
            return true;
        }
        const auto& ssB = substrokes[ ( j + 1 ) % substrokes.size() ];
        if( tooFar( jCurve.endPosition(), ssB.endpoint( false ) ) ) {
            return true;
        }
    }

    return false;
}

} // chains
} // mashup
