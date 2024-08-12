#ifndef MASHUP_STROKEFORWARD_H
#define MASHUP_STROKEFORWARD_H

namespace core {
namespace model {
class Stroke;
} // model
} // core

namespace mashup {

// Bring 'Stroke' into 'blend' namespace.
using Stroke = core::model::Stroke;
using StrokeHandle = const Stroke*;

} // mashup

#endif // #include
