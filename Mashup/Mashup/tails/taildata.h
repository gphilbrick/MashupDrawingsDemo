#ifndef MASHUP_TAILS_TAILDATA_H
#define MASHUP_TAILS_TAILDATA_H

#include <Mashup/endpoint.h>
#include <Mashup/onbarrierpath.h>
#include <Mashup/strokeforward.h>
#include <Mashup/substroke.h>
#include <Mashup/tails/tailtype.h>

#include <Core/model/lineforward.h>
#include <Core/model/curveforward.h>
#include <Core/model/posback.h>
#include <Core/model/polyline.h>

#include <boost/optional.hpp>

namespace mashup {

struct BlendOptions;
class StrokeSegCollider;

namespace tails {

/// Tail-relative information about one end or the other of a 'Stroke' 's'.
/// This is meant to be used by 'TailMaker' internally.
struct TailData
{
    /// Do not call until 'findHitAndOBP' and 'findJoinTo' have been called.
    TailType tailType() const;

    /// What barrier collision is this end of 's' associated with, if any.
    boost::optional< core::model::Pos > barrierHitPos() const;

    /// Only call if 'obpBi' has length > 0.
    void findJoinTo( const Stroke& s,
                    const boost::optional< core::model::Line >& bisector,
                    const BlendOptions& opts,
                    const StrokeSegCollider& coll );

    /// All the polygons in 'polys' are implicitly closed.
    core::model::UniqueCurve joinToFromInflated(
        const core::model::Polylines& polys,
        double& storeCutoffT,
        const core::model::Pos& onWall,
        const core::model::Pos& wallNormal,
        double offsetDist,
        const StrokeSegCollider& coll ) const;

    /// Return a version of 'toLimit' that's cut off where it first hits something in 'coll'.
    core::model::UniqueCurve limitByCollider( const core::model::Curve& toLimit, const StrokeSegCollider& coll ) const;

    void findHitAndOBP( const Stroke& s, const StrokeSegCollider& coll, const BlendOptions& opts );

    /// Only set if tail type is not NoTail.
    /// Represents the part of 's' that can be converted into a tail for this end.
    /// This ends at the appropriate end of 's', e.g., if 'endpoint' is Start, 'tr'
    /// ends at T=0. of 's'.
    boost::optional< Substroke > tr;

    /// same size
    core::model::Polyline trPoly;
    std::vector< double > trPolyT; // T on 's'

    Endpoint endpoint = NumEndpoints;
    /// What T-amount in [0,1] of 's' can in principle be converted into a tail for
    /// this endpoint, assuming none of 's' gets claimed for the other endpoint.
    double tTailAmount = 0.;

    /// An on-barrier path based on finding a barrier position at the endpoint of 's'
    /// (or possibly a padded distance in front of said endpoint) and then walking along
    /// the barriers in both directions.
    OnBarrierPath obpBi;
    /// Identifies the location in 'obp' corresponding to this end of 's'. These two
    /// positions will ideally be equal but, due to padding in the ray-casting used
    /// to find 'obpBi', may be separated by a gap.
    size_t obpBi_startIdx = 0;
    bool turnCW_atHit = false;

    /// The T value in [0,1] on 's' which we will treat as the place where 's' connects
    /// with the start of 'joinTo'. Ideally this is the exact position where 's' crosses
    /// the inflated polyline generated from (parts of) 'obpBi'. However, there are cases
    /// where these two positions will differ somewhat, a subset of those cases where
    /// the endpoint of 's' does not exactly line up with 'obpBi_startIdx' on 'obpBi'.
    double tAtJoinTo = 0.;
    /// Ends at endpoint of 's'.
    core::model::UniqueCurve joinTo;
    /// Radius in canvas space of the circle used to guide tail-based blending operations,
    /// the same radius alluded to in 'BlendOptions'.
    double tailRadius = 0.;
};

} // tails
} // mashup

#endif // #include
