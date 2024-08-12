#ifndef PRINTCURVES_CurvesPostScript_H
#define PRINTCURVES_CurvesPostScript_H

#include <PrintCurves/functors.h>
#include <PrintCurves/strokeproperties.h>

#include <boost/noncopyable.hpp>

#include <array>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

namespace core {
class BSpline2;
class Vector3;
} // core

namespace printCurves {

/// An object for generating a PostScript file containing splines specified in "canvas" space, which is some arbitrary drawing space
/// not strictly connected with the space in the generated PostScript. In canvas space, the origin is meant to appear at the topleft, contrary
/// to PostScript convention.
///
/// The PostScript region and the canvas region it depicts must have the same aspect ratio.
class CurvesPostScript : private boost::noncopyable
{
public:
    using Curve = core::BSpline2;
    using ConstCurve = const Curve&;
    using OutputCurve = std::unique_ptr< core::BSpline2 >;
    using Polycurve = std::vector< const Curve* >;
    using Polyline = std::vector< core::Vector2 >;

    /// In [0,255]
    using RGB = std::array< int, 3 >;

    /// Create an instance meant to draw the parts of splines that fall inside the indicated piece of canvas space.
    /// 'psMinDim' indirectly specifies the dimensions of the PostScript (this is so that the aspect ratio of
    /// the canvas region and the PostScript region will be the same).
    CurvesPostScript( const core::BoundingBoxd& canvas, boost::optional< double > psMinDim = boost::none );

    /// Add the PostScript for the provided curve, which must have degree < 4.
    void addCurve( ConstCurve canvasSpace );
    void addLineSegment( const core::Vector2& a, const core::Vector2& b );

    /// Add the PostScript for a varying width curve whose width is dictated by the provided functor, which outputs
    /// widths in canvas space. 'samplesPerInterval' returns how many samples to take for each C0 subcurve (or how many samples
    /// to take for the entire curve if 'props.treatAsContinuous' is true.
    void addVaryingWidthCurve( ConstCurve canvasSpace, CanvasPosToWidth, SamplesPerInterval samplesPerInterval, const StrokeProperties& props = {} );
    void addVaryingWidthCurve( ConstCurve canvasSpace, ParamToWidth, SamplesPerInterval samplesPerInterval, const StrokeProperties& = {} );

    /// 'canvasSpace' is circle center
    void addCircle( const core::Vector2& canvasSpace, double radiusCanvas, bool strokeOrFill );

    /// Add the PostScript for changing the color unless this would be redundant. The provided are in [0,255].
    void setColor( const RGB& );
    void setColor( int r, int g, int b );
    /// Provide values in [0,1]
    void setColor( const core::Vector3& rgb );
    /// Unless it would be redundant, add the PostScript for changing the line width, provided
    /// in canvas units (not in PostScript units).
    void setLineWidth( double canvasUnits );
    /// Return the current line width in canvas units.
    double lineWidth() const;

    /// Return a string which can be written as an EPS (Encapsulated PostScript) file.
    std::string epsCode() const;

    const core::BoundingBoxd& canvasBounds() const;
private:    

    /// Assuming a path has just been defined and closed, stroke and/or fill it.
    void strokeAndOrFillPath( boost::optional< RGB > strokeColor, boost::optional< RGB > fillColor );

    /// Perform the common functionality of the two public overloads. 'samplesPerInterval' must always return at least 2.
    void addVaryingWidthCurve( ConstCurve canvasSpace, LowLevelWidthFunctor, SamplesPerInterval samplesPerInterval, const StrokeProperties& );

    /// Convert a one-dimensional quantity in canvas space to PostScript dimensions. This
    /// relies on the precondition that the canvas region and the PostScript region have the same
    /// aspect ratio.
    double canvasToPS( double ) const;
    double psToCanvas( double ) const;

    core::Vector2 canvasToPS( const core::Vector2& canvas ) const;
    OutputCurve canvasToPS( ConstCurve canvas ) const;

    /// Generate the EPS for a single curve, assuming it has been converted to the proper space.
    static std::string eps( ConstCurve, bool initialMoveTo, bool strokeAtEnd );

    /// Account for Y-flipping between canvas space and PostScript space.
    static core::Vector2 canvasDirToPS( const core::Vector2& );

    std::stringstream _stream;

    core::BoundingBoxd _canvasBounds;
    core::BoundingBoxd _psBounds;

    RGB _currentRGB;
    double _currentLineWidthPS;
};
	
} // printCurves 

#endif // #include
