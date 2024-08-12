#ifndef MASHUP_RANDOMBINARY_H
#define MASHUP_RANDOMBINARY_H

#include <memory>

namespace mashup {

class RandomBinary
{
public:
    using SeedType = unsigned int;
    RandomBinary();
    ~RandomBinary();
    /// Return true with a probability of 'probOfYes' in [0,1].
    /// Unless given different seeds, two of these should return
    /// the exact same sequence of answers given the same inputs.
    bool yes( double probOfYes );
    void seed( SeedType );

    static SeedType defaultSeed;
private:
    struct Imp;
    const std::unique_ptr< Imp > _imp;
};

} // mashup

#endif // #include
