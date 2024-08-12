#ifndef MASHUP_STROKESEGCOLLIDER_H
#define MASHUP_STROKESEGCOLLIDER_H

#include <Mashup/drawingid.h>
#include <Mashup/samedrawinghits.h>
#include <Mashup/strokeforward.h>

#include <Core/model/lineforward.h>
#include <Core/model/polyline.h>
#include <Core/math/segcollidergrid.h>

#include <boost/optional.hpp>

#include <map>

namespace mashup {

class Drawings;
struct OnBarrierPath;
struct StrokePoly;
struct Substroke;

struct StrokeSegColliderMetadata
{
    using SegID = int;

    StrokeHandle stroke = nullptr;
    /// A segment in the collider represents 't' along 'stroke'.
    std::array< double, 2 > t = { 0., 0. };
    /// Of the 'Stroke' outline that this segment is a part of.
    core::model::Pos normal;
    /// Can be checked on its own for 'this' equality.
    SegID segID = 0;

    /// Is this segment the cap of 'stroke', not part of one of its sides.
    bool isCap = false;

    /// These define a sequence of segments representing one side of 'stroke'.
    boost::optional< SegID > next;
    boost::optional< SegID > prev;
};

/// A spatial structure for rapidly finding out whether a line segment interects any of a collection
/// of line segments taken from 'Stroke's that are participating in a blend-drawings operation.
class StrokeSegCollider : public core::math::SegColliderGrid< StrokeSegColliderMetadata >
{
public:
    using Base = core::math::SegColliderGrid< StrokeSegColliderMetadata >;
    using Metadata = StrokeSegColliderMetadata;

    /// Collision between some directed 1D object (segment or ray) and
    /// a registered segment.
    struct Hit
    {
        /// [0,1] number indicating dist along the 1D object that collided with the stroke graph.
        double fHitter = 0.;
        double strokeT = 0.;
        SegWithData swd;
        /// Lies on the offset curve of some 'Stroke', within floating point error
        core::model::Pos pos;
    };

    StrokeSegCollider( const core::model::BoundingBox& canvasBounds );

    void addStroke( const StrokePoly& );
    /// Remove any segments associated w/ this 'Stroke'.
    void removeStroke( StrokeHandle );

    /// Return whether 'hitter' has any hits ('Stroke' and on-'Stroke' T) that pass 'testStrokeAndT'.
    /// Include backwards/forwards collisions.
    bool hitsAnythingPassing( const core::model::Polyline& hitter,
                              std::function< bool( StrokeHandle, double ) > testStrokeAndT ) const;
    bool hitsAnythingPassing( const core::model::Polyline& hitter,
                              std::function< bool( const SegWithData& ) > testSWD ) const;
    bool hitsAnything( const core::model::Polyline& hitter ) const;
    
    /// Return the first collision along 'hitter' with something satisfying 'pred' (if any; return boost::none
    /// if there is no first collision). If 'ignoreFromBehind' is true, ignore collisions with back-facing
    /// barriers (as indicated by barriers' normals relative to 'hitter').
    boost::optional< Hit > firstHit( const core::model::Seg& hitter, SWDPredicate pred, bool ignoreFromBehind ) const;
    /// Return in no particular order all the collisions found along 'hitter'.
    std::vector< Hit > allHits( const core::model::Seg& hitter, SWDPredicate pred, bool ignoreFromBehind ) const;

    /// Return either a valid path of length >1 based on pathfinding from the on-segment location
    /// that 'hitter' first hits. Return empty path in event of failure, e.g., if 'hitter' hits nothing.
    ///
    /// If 'bothDirs' is true, do path finding in both directions and stitch the result together, using
    /// 'storeStartIndex' to indicate to the caller where in 'ret' corresponds to where 'hitter' hit.
    /// Increasing index from this point means walking along the path in the direction suggested
    /// by 'hitter'.
    ///
    /// If 'bothDirs' is false, only do path finding in the direction suggested by the direction of 'hitter'
    /// (specifically, how it relates to the normal of the segment it hits).
    OnBarrierPath onBarrierPath( const core::model::Seg& hitter, bool bothDirs, size_t& storeStartIndex ) const;
    /// Set 'resultIsClosed' if the return path goes all the way around (first/last points will
    /// be equal in that case).
    OnBarrierPath onBarrierPath( const Hit& start, bool goWithBarr ) const;

    std::vector< SegWithData > strokeSegsWithinRange( const Pos& posCanvas, double range ) const;
    std::vector< SegWithData > strokeSegsWithinRange( const IPos&, double range ) const;

    /// Fill 'store' with information about original-drawing 'Stroke's hitting each other.
    /// 'd' tells 'this' which 'Stroke'-segments belong to which 'Drawing's.
    void sameDrawingHits( DrawingToSameDrawingHits& store, const Drawings& d ) const;
private:
    StrokeSegColliderMetadata::SegID _nextSegID;
    // this is just to make removeStroke faster
    /// Track which array coordinates are involved in representing each 'Stroke'.
    std::map< StrokeHandle, SetOfIPos > _strokeToInvolvedCoords;
    std::map< StrokeSegColliderMetadata::SegID, SegWithData > _idToSWD;
};

} // mashup

#endif // #include
