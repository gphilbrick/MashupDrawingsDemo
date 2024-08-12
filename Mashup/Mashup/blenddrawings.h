#ifndef MASHUP_BLENDDRAWINGS_H
#define MASHUP_BLENDDRAWINGS_H

#include <Mashup/drawings.h>
#include <Mashup/strokepoly.h>

#include <Core/model/posforward.h>
#include <Core/model/strokesforward.h>

#include <map>
#include <memory>

namespace core {
namespace view {
class ProgressBar;
} // view
} // core

namespace mashup {

struct BlendOptions;
class Drawings;
class SameDrawingHits;
class StrokeSegCollider;

/// Takes in "original" 'Drawing's (collections of 'Stroke's) and produces a
/// blend-drawings result in the form of 'Stroke's that visually blend the two inputs.
class BlendDrawings
{
public:
    using StrokeToPoly = std::map< StrokeHandle, StrokePoly >;

    BlendDrawings( Drawings&&, const BlendOptions&, core::view::ProgressBar* progBar );
    ~BlendDrawings();
    /// Call only once.
    void perform();
    const core::model::UniqueStrokes& result() const;

    const BlendOptions& options() const;
    const Drawings& drawings() const;
    /// Return the collider representing only original-drawing 'Stroke's.
    const StrokeSegCollider& collAB() const;
    /// Return a mapping from original-drawing 'Stroke' 's' to 'this''s polygon approximation of 's'.
    const StrokeToPoly& originalStrokeToPoly() const;
    const SameDrawingHits& sameDrawingHits( DrawingID ) const;

    /// Return whether 'p' is inside one of the 'Stroke's of one of the original drawings.
    bool insideOriginalStroke( const core::model::Pos& p ) const;

    /// Return how many points the (spine of a) polyline-based 'Stroke' approximation of 's'
    /// should be.
    size_t strokePolyLength( const Stroke& s ) const;
private:
    struct Imp;
    const std::unique_ptr< Imp > _imp;
};

} // mashup

#endif // #include
