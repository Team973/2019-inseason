/*
 * Wave.cpp
 *
 *  Created on: February 13, 2018
 *      Author: Cole Brinsfield
 */

#include "lib/pixelprocessors/Wave.h"

namespace frc973 {

namespace LightPattern {

Wave::Wave(Color background, Color foreground, int period) {
    this->m_foreground = foreground;
    this->m_background = background;
    this->m_period = period;
}

void Wave::Tick(PixelState& state) {
    for (unsigned int i = 0; i < state.numLEDs; i++) {
        state.pixels.at(i) = m_background;
    }

    for (unsigned int i = 0; i < state.numLEDs; i++) {
        state.pixels.at(i) = m_background.gradientTo(
            m_foreground,
            (cos((i + state.frame) * (2 * M_PI / m_period)) + 1) / 2);
    }
}
}
}
