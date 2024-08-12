#ifndef CORE_MODEL_STROKE_H
#define CORE_MODEL_STROKE_H

#include <Core/model/curveback.h>
#include <Core/model/polyline.h>
#include <Core/model/strokesforward.h>

#include <boost/core/noncopyable.hpp>

#include <memory>

namespace core {
class CurveInterval;
} // core

namespace core {
namespace model {

/// A path and a definition of width along its length.
class Stroke : private boost::noncopyable
{
public:
    using WidthCurve = Curve;

    explicit Stroke( double width );
    explicit Stroke( const WidthCurve& width );
    explicit Stroke( std::unique_ptr< WidthCurve >&& width );

    std::unique_ptr< Stroke > clone() const;

    /// Return a handle to the position curve or "spine" of 'this'.
    const Curve& curve() const;   
    void setCurve( const Curve& curveToCopy );
    void setCurve( UniqueCurve&& curveToOwn );

    /// Return whether this stroke should be treated as a point. The curve must be set.
    bool zeroLength() const;
    /// Return whether the endpoints match.
    bool closed() const;
    /// Return a bounding box guaranteed to encompass all of the stroke, _including a fudge factor to account for width_.
    core::BoundingBoxd boundingBox() const;

    /// Create a Stroke representing part of this Stroke. If 'i' is only applicable to a closed curve, then this stroke must be closed.
    std::unique_ptr< Stroke > strokeInterval( const core::CurveInterval& i ) const;
    /// Assume that the desired T-interval is increasing if 'tStart' < 'tEnd' and decreasing otherwise.
    std::unique_ptr< Stroke > strokeInterval( double tStart, double tEnd ) const;

    std::unique_ptr< Stroke > reverse() const;

    /// Return the width of the 'Stroke' in canvas space at 't' in [0,1].
    double width( double t ) const;
    double maxWidth() const;
    const WidthCurve& widthCurve() const;
private:
    UniqueCurve _curve;
    std::unique_ptr< const WidthCurve > _width;
};

} // model
} // core

#endif // #include guard
