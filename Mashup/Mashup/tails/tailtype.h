#ifndef MASHUP_TAILS_TAILTYPE_H
#define MASHUP_TAILS_TAILTYPE_H

namespace mashup {
namespace tails {

/// What kind of tail should 'Stroke' 's' have at one of its ends?
enum TailType
{
    /// This end of 's' should not be tail-ified in any way.
    NoTail,
    /// A tail is desired for this end of 's' but it will have to be
    /// a simple tapering to width=0; there is not enough data to do
    /// a real tail.
    FallbackTail,
    /// A tail is desired for this end of 's' and there is enough
    /// data to support its generation.
    NormalTail
};

} // tails
} // mashup

#endif // #include
