#ifndef MASHUPDEMO_MASHUPINPUTS_H
#define MASHUPDEMO_MASHUPINPUTS_H

#include <Mashup/abfield.h>
#include <Mashup/blendoptions.h>
#include <Mashup/drawings.h>

#include <Core/utility/boundingbox.h>

#include <memory>
#include <string>

namespace mashupDemo {

/// The inputs to a demonstration mashup-drawings scenario.
struct MashupInputs
{
    /// Not used in all scenarios.
    std::unique_ptr< mashup::ABField > abField;

    /// Used to construct the paths for saving input/output images.
    std::string name;

    mashup::Drawings inputDrawings;
    mashup::BlendOptions options;

    /// The canvas-space rectangle in which the mashup takes place (this maps to
    /// the boundaries of the exported input/output images.)
    core::BoundingBoxd canvasBounds;
};

} // mashupDemo

#endif // #include
