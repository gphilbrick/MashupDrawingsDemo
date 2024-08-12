#ifndef MASHUP_WEIGHTFUNCTOR_H
#define MASHUP_WEIGHTFUNCTOR_H

#include <Mashup/abfield.h>
#include <Mashup/strokeforward.h>

#include <functional>

namespace mashup {

class BlendDrawings;

class WeightFunctor
{
public:
    using StrokeT = double;
    /// Return a >=0 weight associated with T='t' on original-drawing 'Stroke' 's'.
    virtual double weight( const Stroke& s, StrokeT t, const BlendDrawings& ) const = 0;
};

class WeightFunctor_WidthBased : public WeightFunctor
{
public:
    double weight( const Stroke&, StrokeT, const BlendDrawings& ) const override;
};

class WeightFunctor_ABFieldBased : public WeightFunctor
{
public:
    WeightFunctor_ABFieldBased( const ABField& );
    double weight( const Stroke&, StrokeT, const BlendDrawings& ) const override;
private:
    const ABField& _abField;
};

} // mashup

#endif // #include
