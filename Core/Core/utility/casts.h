#ifndef CORE_UTILITY_CASTS_H
#define CORE_UTILITY_CASTS_H

#include <memory>
#include <set>
#include <vector>

namespace core {

// From https://code-examples.net/en/q/1431941
template< typename Derived, typename Base >
std::unique_ptr< Derived > dynamicUniqueToDerived( std::unique_ptr< Base >&& base )
{
    Derived* const derivedRaw = dynamic_cast< Derived* >( base.release() );
    return std::unique_ptr< Derived >( derivedRaw );
}

template< typename T >
std::vector< const T* > uniquesToConstRaws( const std::vector< std::unique_ptr< T > >& smart )
{
    std::vector< const T* > raws( smart.size() );
    for( size_t i = 0; i < smart.size(); i++ ) {
        raws[ i ] = smart[ i ].get();
    }
    return raws;
}

template< typename T >
std::vector< const T* > sharedToConstRaws( const std::vector< std::shared_ptr< const T > >& smart )
{
    std::vector< const T* > raws( smart.size() );
    for( size_t i = 0; i < smart.size(); i++ ) {
        raws[ i ] = smart[ i ].get();
    }
    return raws;
}

template< typename T >
std::vector< const T* > sharedToConstRaws( const std::vector< std::shared_ptr< T > >& smart )
{
    std::vector< const T* > raws( smart.size() );
    for( size_t i = 0; i < smart.size(); i++ ) {
        raws[ i ] = smart[ i ].get();
    }
    return raws;
}

template< typename T >
std::set< const T* > sharedToConstRaws( const std::set< std::shared_ptr< const T > >& smart )
{
    std::set< const T* > raws;
    for( auto& fromSmart : smart ) {
        raws.emplace( fromSmart.get() );
    }
    return raws;
}

template< typename Child, typename Base >
std::vector< const Base* > upcastRaws( const std::vector< const Child* >& child )
{
    std::vector< const Base* > parent( child.size() );
    for( size_t i = 0; i < parent.size(); i++ ) {
        parent[ i ] = child[ i ];
    }
    return parent;
}

template< typename T >
std::set< std::shared_ptr< const T > > constConvert( const std::set< std::shared_ptr< T > >& source )
{
    std::set< std::shared_ptr< const T > > toReturn;
    for( const auto& fromSource : source ) {
        toReturn.emplace( fromSource );
    }
    return toReturn;
}

} // core

#endif // #include
