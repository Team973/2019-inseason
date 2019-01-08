/*
 * CenterMirror.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: Cole Brinsfield
 */

#include "CenterMirror.h"

namespace LightPattern {

CenterMirror::CenterMirror(PixelStateProcessor* processor, bool inverse) {
    this->processor = processor;
    this->m_inverse = inverse;
}

void CenterMirror::Tick(PixelState& state) {
    m_previousNumLEDs = state.numLEDs;
    state.numLEDs = state.numLEDs / 2;
    PixelStateProcessorModulator::Tick(state);
    if (m_inverse) {
        PushToRightSide(state);
        std::reverse(state.pixels.begin(),
                     state.pixels.begin() + m_previousNumLEDs / 2);
    }
    else {
        PushToRightSide(state);
        std::reverse(state.pixels.begin() + m_previousNumLEDs / 2,
                     state.pixels.end());
    }
    state.numLEDs = m_previousNumLEDs;
}

void CenterMirror::PushToRightSide(PixelState& state) {
    for (std::size_t i = 0; i < m_previousNumLEDs / 2; i++) {
        state.pixels.at(m_previousNumLEDs / 2 + i) = state.pixels.at(i);
    }
}
}
