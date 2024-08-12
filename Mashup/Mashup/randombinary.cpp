#include <randombinary.h>

#include <random>

namespace mashup {

RandomBinary::SeedType RandomBinary::defaultSeed = 0;

using Gen = std::mt19937;

struct RandomBinary::Imp
{
    Gen gen;
};

RandomBinary::RandomBinary() : _imp( std::make_unique< Imp >() )
{
    seed( defaultSeed );
}

void RandomBinary::seed( SeedType seed )
{
    static_assert( std::is_same< SeedType, Gen::result_type >::value, "Wrong SeedType" );
    _imp->gen = std::mt19937{ seed };
}

RandomBinary::~RandomBinary()
{
}

bool RandomBinary::yes( double probOfYes )
{
    // <X> from https://stackoverflow.com/questions/43329352/generating-random-boolean
    std::bernoulli_distribution dist( probOfYes );
    return dist( _imp->gen );
}

} // mashup
