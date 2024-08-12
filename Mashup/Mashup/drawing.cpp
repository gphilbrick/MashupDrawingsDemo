#include <drawing.h>

#include <strokeback.h>

#include <Core/exceptions/runtimeerror.h>

#include <Core/utility/casts.h>

namespace mashup {

Drawing::Drawing()
{}

Drawing::Drawing( Drawing&& other )
{
    _strokes = std::move( other._strokes );
    _handles = std::move( other._handles );
    _handleToIndex = std::move( other._handleToIndex );
}

Drawing::~Drawing()
{}

void Drawing::addStroke( UniqueStroke&& toOwn )
{
    _strokes.push_back( std::move( toOwn ) );
    const auto* const strokeHandle = _strokes.back().get();
    _handles.emplace( strokeHandle );
    _handleToIndex[ strokeHandle ] = _strokes.size() - 1;
}

size_t Drawing::numStrokes() const
{
    return _strokes.size();
}

const Stroke* Drawing::stroke( size_t i ) const
{
    return _strokes[ i ].get();
}

void Drawing::forEach( std::function< void( const Stroke& ) > f ) const
{
    for( const auto& s : _strokes ) {
        f( *s );
    }
}

bool Drawing::contains( StrokeHandle s ) const
{
    return _handles.find( s ) != _handles.end();
}

size_t Drawing::index( StrokeHandle s ) const
{
    const auto it = _handleToIndex.find( s );
    if( it == _handleToIndex.end() ) {
        THROW_UNEXPECTED;
    } else {
        return it->second;
    }
}

} // mashup
