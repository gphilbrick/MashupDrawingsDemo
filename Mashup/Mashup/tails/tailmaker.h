#ifndef MASHUP_TAILS_TAILMAKER_H
#define MASHUP_TAILS_TAILMAKER_H

#include <Mashup/strokeforward.h>

#include <Core/model/lineforward.h>

#include <Core/utility/boundinginterval.h>

#include <boost/optional.hpp>

#include <memory>

namespace mashup {

class BlendDrawings;
class StrokeSegCollider;

namespace tails {

/// Takes in a 'Stroke' and converts one or both of its ends into "tails".
class TailMaker
{
public:
    /// For a blend-stroke in progress 's', this is either a non-zero-length T-interval in
    /// [0,1] indicating what part of 's' (possibly all of it) must not be converted into
    /// tails, or it is boost::none, meaning all of 's' may be converted into tails.
    using PreserveInterval = boost::optional< core::BoundingIntervald >;

    using Coll = StrokeSegCollider;
    /// 'preserveMid' in [0,1] indicates indirectly which ends of 'stroke' should be converted
    /// into tails (e.g., if it is {0,0.5}, then only the T=1 end will have a tail).
    /// If 'preserveMid' is boost::none, both ends may be converted to tails.
    TailMaker(
        const Stroke& stroke,
        const PreserveInterval& preserveMid,
        const Coll& collider,
        const BlendDrawings& );
    ~TailMaker();
    std::unique_ptr< Stroke > result();
private:
    struct Imp;
    const std::unique_ptr< Imp > _imp;
};

} // tails
} // mashup

#endif // #include
