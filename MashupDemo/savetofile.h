#ifndef MASHUPDEMO_SAVETOFILE_H
#define MASHUPDEMO_SAVETOFILE_H

#include <Mashup/drawings.h>

#include <Core/utility/boundingbox.h>

#include <filesystem>

namespace mashupDemo {

/// Save an EPS vector image that shows the two input 'drawings'.
/// Use 'canvasBounds' as the view window.
void saveInputDrawingsEPS(
    const mashup::Drawings&,
    const std::filesystem::path&,
    const core::BoundingBoxd& canvasBounds );

/// Save an EPS vector image showing 'strokes' viewed within 'canvasBounds'.
void saveMashedUpDrawingEPS(
    const std::vector< std::unique_ptr< core::model::Stroke > >& strokes,
    const std::filesystem::path&,
    const core::BoundingBoxd& canvasBounds );

} // mashupDemo	

#endif // #include
