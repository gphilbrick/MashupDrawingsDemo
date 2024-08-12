#include <chains/nextstep.h>

#include <strokeback.h>

#include <Core/exceptions/runtimeerror.h>
#include <Core/model/stroketools.h>

namespace mashup {
namespace chains {

std::unique_ptr< NextStep > NextStep::clone() const
{
    auto ret = std::make_unique< NextStep >();
    ret->next = next;
    ret->nextTrimmed = nextTrimmed;
    ret->prevTrimmed = prevTrimmed;
    ret->joint = joint ? joint->clone() : nullptr;
    return ret;
}

std::unique_ptr< NextStep > NextStep::reverse( const Substroke& prev ) const
{
    auto ret = std::make_unique< NextStep >();
    ret->joint = joint ? joint->reverse() : nullptr;
    ret->next = prev.reverse();
    ret->nextTrimmed = prevTrimmed.reverse();
    ret->prevTrimmed = nextTrimmed.reverse();
    return ret;
}

std::unique_ptr< Stroke > NextStep::midStroke() const
{
    if( joint ) {
        return joint->clone();
    } else {
        // All of 'this' is from same 'Stroke'
        if( prevTrimmed.stroke != nextTrimmed.stroke ) {
            THROW_UNEXPECTED;
        }
        const auto* const stroke = prevTrimmed.stroke;
        return stroke->strokeInterval( prevTrimmed.t[ 1 ], nextTrimmed.t[ 0 ] );
    }
}

std::unique_ptr< Stroke > NextStep::asStroke() const
{
    if( joint ) {
        auto prevStroke = prevTrimmed.asStroke();
        auto nextStroke = nextTrimmed.asStroke();

        core::model::RawConstStrokes toStitch{ prevStroke.get() };
        toStitch.push_back( joint.get() );
        toStitch.push_back( nextStroke.get() );

        // It's possible that this thing actually is closed, but I don't
        // think that will matter where we're going to use this.
        return core::model::stitchC0Strokes( toStitch, false );
    } else {
        const auto* const stroke = prevTrimmed.stroke;
        return stroke->strokeInterval( prevTrimmed.t[ 0 ], nextTrimmed.t[ 1 ] );
    }
}

std::unique_ptr< NextStep > NextStep::singleIntervalCase( const Substroke& stubA, const Substroke& stubB )
{
    auto ret = std::make_unique< NextStep >();
    ret->prevTrimmed = stubA;
    ret->nextTrimmed = stubB.reverse();
    ret->next = ret->nextTrimmed;
    return ret;
}

} // chains
} // mashup
