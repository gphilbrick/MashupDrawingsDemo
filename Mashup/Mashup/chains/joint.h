#ifndef MASHUP_CHAINS_JOINT_H
#define MASHUP_CHAINS_JOINT_H

#include <Mashup/strokeforward.h>

#include <memory>

namespace mashup {
namespace chains {

using Joint = std::unique_ptr< Stroke >;

} // chains
} // mashup

#endif // #include
