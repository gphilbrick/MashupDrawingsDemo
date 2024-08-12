#ifndef MASHUP_SUBSTROKE_H
#define MASHUP_SUBSTROKE_H

#include <Mashup/strokeforward.h>

#include <Core/model/posforward.h>

#include <array>
#include <functional>
#include <memory>

namespace mashup {

/// A directed interval along some 'Stroke'.
struct Substroke
{
    /// Contract of < function
    using CompFunc = std::function< bool( const Substroke&, const Substroke& ) >;

    Substroke();
    Substroke( const Stroke& s, double tA, double tB );

    Substroke reverse() const;
    bool tIncreasing() const;
    std::unique_ptr< Stroke > asStroke() const;
    core::model::Pos endpoint( bool endOrStart ) const;
    double endWidth( bool endOrStart ) const;
    /// Return the normalized direction of the 'Substroke' (might be reverse of
    /// the 'Stroke' itself) at one end or the other.
    core::model::Pos endDirNormalized( bool endOrStart ) const;

    /// If 'start' is true, move start T to 'trimT' if 'trimT' is after current start T.
    /// If 'start' is false, move end T to 'trimT' if 'trimT' is before current end T.
    ///
    /// "After" and "before" here are relative to whether 'this' is T-increasing or T-decreasing.
    /// The trim should not flip the direction of 'this', e.g., 'trimT' must come before current
    /// end T if 'start' is true.
    ///
    /// The trim must also not reduce the length of 'this' to 0 (and 'this' must not currently
    /// have length = 0).
    void nonFlippingTrim( double trimT, bool start );

    /// 'fA' != 'fB' in [0,1]. May be ordered whichever way.
    Substroke interval( double fA, double fB ) const;
    /// Given 'tStroke', a T value from 'stroke' (assumed to lie within the 't' interval
    /// of 'this'), return a corresponding F value in [0,1] relative to 'this'.
    double f( double tStroke ) const;
    /// Given 'f' in [0,1] relative to 'this', return the corresponding T value on 'stroke'.
    double tFromF( double f ) const;

    /// Return whether the end ('atEndOrStart'=true) or start (...=false) of 'this'
    /// is the endpoint (T=0/T=1) of 'stroke'.
    bool includesStrokeEndpoint( bool atEndOrStart ) const;
    bool contains( double t ) const;
    bool operator == ( const Substroke& b ) const;
    bool operator != ( const Substroke& b ) const;

    /// Return standard < ordering treating 'a' and 'b' as two numerical triplets (stroke-handle, tStart, tEnd).
    static bool compare_standard( const Substroke& a, const Substroke& b );

    /// non-null
    StrokeHandle stroke;
    /// increasing or decreasing.
    /// [0] is T at start of 'this', [1] is T at end.
    std::array< double, 2 > t;
};

} // mashup

#endif // #include
