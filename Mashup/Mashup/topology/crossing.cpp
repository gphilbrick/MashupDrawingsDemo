#include <topology/crossing.h>

#include <Core/exceptions/runtimeerror.h>

#include <algorithm>

namespace mashup {
namespace topology {

using Stub = Crossing::Stub;

Crossing::Crossing()
    : _originalConnections( Substroke::compare_standard )
{
}

void Crossing::add( const Stub& s )
{
    _stubs.push_back( s );
}

void Crossing::add( const Stub& stubA, const Stub& stubB )
{
    if( stubA.stroke != stubB.stroke ) {
        THROW_RUNTIME( "Unexpected inputs" );
    }

    _originalConnections.emplace( stubA, stubB );
    _originalConnections.emplace( stubB, stubA );
    add( stubA );
    add( stubB );
}

const std::vector< Stub >& Crossing::stubs() const
{
    return _stubs;
}

bool Crossing::originallyConnected( const Stub& a, const Stub& b ) const
{
    const auto it = _originalConnections.find( a );
    if( it != _originalConnections.end() ) {
        return it->second == b;
    } else {
        return false;
    }
}

boost::optional< Stub > Crossing::originalConnection( const Stub& stub ) const
{
    const auto it = _originalConnections.find( stub );
    if( it != _originalConnections.end() ) {
        return it->second;
    }
    return boost::none;
}

void Crossing::addEnvelopeAroundOccluded( const Substroke& env )
{
    _envelopesAroundOccluded.push_back( env );
}

bool Crossing::isPartOf( StrokeHandle s, double t ) const
{
    for( const auto& ss : _envelopesAroundOccluded ) {
        if( ss.stroke == s ) {
            if( ss.contains( t ) ) {
                return true;
            }
        }
    }
    return false;
}

} // topology
} // mashup
