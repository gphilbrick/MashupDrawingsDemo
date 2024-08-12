#include <utility/boundinginterval.h>

namespace core {

std::vector< BoundingIntervald > removeBoundingInterval(
    const BoundingIntervald& removeFrom, const BoundingIntervald& remove, double epsilon )
{
    const bool fromMinContained = remove.contains( removeFrom.min() );
    const bool fromMaxContained = remove.contains( removeFrom.max() );
    if( fromMinContained && fromMaxContained ) {
        return {};
    } else if( removeFrom.contains( remove.min() ) || removeFrom.contains( remove.max() ) ) {
        std::vector< BoundingIntervald > toReturn;
        if( !fromMinContained ) {
            const BoundingIntervald toAdd( removeFrom.min(), remove.min() );
            if( toAdd.length() >= epsilon ) {
                toReturn.push_back( toAdd );
            }
        }
        if( !fromMaxContained ) {
            const BoundingIntervald toAdd( remove.max(), removeFrom.max() );
            if( toAdd.length() >= epsilon ) {
                toReturn.push_back( toAdd );
            }
        }
        return toReturn;
    } else {
        return { removeFrom };
    }
}

} // core
