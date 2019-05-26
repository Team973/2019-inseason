/*
 * ReverseModifier.cpp
 *
 *  Created on: March 11, 2018
 *      Author: Cole Brinsfield
 */

#include "lib/pixelprocessors/ReverseModifier.h"

namespace frc973 {

namespace LightPattern {
ReverseModifier::ReverseModifier(PixelStateProcessor* processor) {
    this->processor = processor;
}
void ReverseModifier::Tick(PixelState& state) {
    PixelStateProcessorModulator::Tick(state);
    std::reverse(state.pixels.begin(), state.pixels.end());
}
}
}
