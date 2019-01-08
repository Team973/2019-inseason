/*
 * LoopModulator.cpp
 *
 *  Created on: Feb 10, 2018
 *      Author: Cole Brinsfield
 */

#include "LoopModulator.h"

namespace LightPattern {
LoopModulator::LoopModulator(PixelStateProcessor* processor) {
    this->processor = processor;
}

void LoopModulator::Tick(PixelState& state) {
    PixelStateProcessorModulator::Tick(state);

    // every frame, rotate (frame % count) times, so that
    // it completes a rotation every (count) frames
    int amount = state.frame % state.numLEDs;
    this->Rotate(state.pixels, amount, state.numLEDs);
}

void LoopModulator::Rotate(std::vector<Color>& pixels, int amount,
                           int numLEDs) {
    // rotation has to be inline, but since that's
    // pretty much impossible we make a copy
    std::vector<Color> copy = std::vector<Color>(pixels.begin(), pixels.end());

    for (int i = 0; i < numLEDs; i++) {
        int newIndex;
        if (i + amount >= numLEDs) {
            newIndex = amount - (numLEDs - i);
        }
        else {
            newIndex = i + amount;
        }
        pixels.at(newIndex) = copy[i];
    }
}
}
