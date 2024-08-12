#ifndef MASHUP_BLENDOPTIONS_H
#define MASHUP_BLENDOPTIONS_H

#include <Mashup/weightfunctor.h>
#include <Mashup/drawingid.h>

#include <Core/utility/boundinginterval.h>

#include <boost/optional.hpp>

#include <functional>
#include <memory>

namespace mashup {

struct BlendOptions
{
    struct RoutingOptions
    {
        std::unique_ptr< WeightFunctor > wFunctor
            = std::make_unique< WeightFunctor_WidthBased >();

        /// In [0,1]. When this is 1., the probability 'prob' of choosing the second connect-to option is unchanged;
        /// as this approaches 0, 'prob' gets pulled more sharply away from 50% toward either 100% or 0%.
        /// If this is 0, 'prob' is forced to be either 100% or 0%, unless it is exactly 50%.
        double flipVoteGamma = 1.;

        /// Roughly speaking, what "turning radius" do we want when trimming 'Stub's to create joints?
        double jointRad = 50.;
    };

    struct TailOptions
    {
        /// in canvas space, >0
        /// The maximum value to use for a tail's "radius": the radius of the circle
        /// used to determine the extent of tail-related blending operations.
        double maxRad_canvas = 60.;
        /// in (0,1] (relative to tail radius, which is not necessarily 'maxRad_canvas')
        /// A tail should be this far away from barriers.
        double maxOffsetDist_f = 0.2;
        /// in (0,1) (relative to k)
        /// If a tail is k away from barriers, its width <= k * this.
        double maxWidthFillAllowed_f = 0.1;
        /// Relative to tail radius, which is not necessarily 'maxRad_canvas'.
        /// After a tail has left the tail circle, it can travel this far (times
        /// tail radius) before ending.
        double maxOutsideCircle_f = 0.8;
    };

    /// If set (to A or B), then perform a blend-drawings operation that keeps
    /// that drawing visually unchanged.
    boost::optional< DrawingID > preserveDrawing;

    /// Used to narrow short, stubby blendstrokes.
    double minLengthToWidth = 3.;

    RoutingOptions routing;
    TailOptions tails;
};

} // mashup

#endif // #include
