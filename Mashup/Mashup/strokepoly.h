#ifndef MASHUP_STROKEPOLY_H
#define MASHUP_STROKEPOLY_H

#include <Mashup/drawingid.h>
#include <Mashup/strokeforward.h>
#include <Mashup/strokeside.h>

#include <Core/model/boundingboxback.h>
#include <Core/model/lineforward.h>
#include <Core/model/polyline.h>

#include <Core/utility/boundingbox.h>

#include <functional>

namespace mashup {

/// A (possibly self-intersecting) polygon approximating the outline (but not necessarily the silhouette, obviously) of a 'Stroke'.
struct StrokePoly
{
    using Normal = core::model::Pos;
    using Normals = core::model::Polyline;

    StrokePoly();
    /// Let 'affil' indicate which 'Drawing' (if any) 's' is associated with along its entire length.
    /// 'numPoints' is a guide; do not expect it to be exactly honored in result.
    StrokePoly( const Stroke& s, size_t numPoints );
    /// 'numPoints' is a guide; do not expect it to be exactly honored in result.
    void initFrom( const Stroke& s, size_t numPoints );
    /// Does 'this' represent a closed 'Stroke'?
    bool closed() const;
    bool outlineCrosses( const core::model::Seg& seg ) const;
    bool contains( const core::model::Pos& p ) const;

    /// Return a point representing moving 'seekT' (in [0,1]) alone the indicated side.
    core::model::Pos onSide( double seekT, StrokeSide sideIdx ) const;

    /// Call 'f...' on all the segments (in no particular order) of the polygon.
    /// f's arguments
    ///     (0) segment
    ///     (1) normal of segment that (locally) points outside the 'Stroke'
    ///     (2) stroke T at start of segment
    ///     (3) stroke T at end of segment
    ///     (4) drawing affiliation of the segment
    void forEachSeg( std::function< void( const core::model::Seg&, const Normal&, double, double ) > f_seg_norm_tA_tB ) const;
    size_t pointsPerSide() const;
    bool participates() const;

    /// Return a collection of T values (in [0,1], relative to start and end of 'this') reflecting where
    /// 'hitter' crosses the outline.
    std::vector< double > hitTs( const core::model::Polyline& hitter ) const;
    /// Same idea, but short-circuits to true on first hit.
    bool hitsAtAll( const core::model::Polyline& hitter ) const;

    StrokeHandle stroke = nullptr;

    /// Both polylines in 'sides' have same size (>1 if 'this' "participates"). Points ordered from T=0 to T=1.
    core::model::Polyline sides[ NumSides ];
    /// Both have size one less than the size of one of 'sides'.
    Normals sideNormals[ NumSides ];
    /// 0.0 to 1.0
    std::vector< double > t;
    core::model::BoundingBox bounds;

    /// Only used for open 'stroke'.
    boost::optional< Normal > capNormal_T0;
    boost::optional< Normal > capNormal_T1;

    /// A crude way of ending 'forEachSeg' early.
    mutable bool killForEachSeg = false;
private:
    void init_open( size_t numPoints );
    void init_closed( size_t numPoints );
};
using StrokePolys = std::vector< StrokePoly >;
using StrokePolyHandles = std::vector< const StrokePoly* >;

} // mashup

#endif // #include
