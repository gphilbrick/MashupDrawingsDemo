#ifndef MASHUP_DRAWING_H
#define MASHUP_DRAWING_H

#include <Mashup/strokeforward.h>

#include <boost/noncopyable.hpp>

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <vector>

namespace mashup {

/// A collection of (and, code-wise, literal owner of) some "original-drawing" 'Stroke's.
class Drawing : private boost::noncopyable
{
public:
    using UniqueStroke = std::unique_ptr< Stroke >;
    Drawing();
    Drawing( Drawing&& );
    ~Drawing();
    void addStroke( UniqueStroke&& toOwn );
    StrokeHandle stroke( size_t ) const;
    bool contains( StrokeHandle ) const;
    /// 's' must belong to 'this'.
    size_t index( StrokeHandle s ) const;
    size_t numStrokes() const;
    /// Iterate over the 'Stroke's in the order they were add()ed.
    void forEach( std::function< void( const Stroke& ) > ) const;
private:
    std::vector< UniqueStroke > _strokes;
    std::set< StrokeHandle > _handles;
    std::map< StrokeHandle, size_t > _handleToIndex;
};

} // mashup

#endif // #include
