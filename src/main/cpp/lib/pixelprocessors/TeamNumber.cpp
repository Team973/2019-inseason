/*
 * TeamNumber.cpp
 *
 *  Created on: Feb 8, 2018
 *      Author: Cole Brinsfield
 */

#include "TeamNumber.h"

namespace LightPattern {

TeamNumber::TeamNumber() : TeamNumber(Color{255, 0, 0}, Color{0, 0, 255}) {
}
TeamNumber::TeamNumber(Color foreground, Color background)
        : m_foreground(foreground), m_background(background) {
}

void TeamNumber::Tick(PixelState& state) {
    int spacing = (state.numLEDs - 9 - 7 - 3) / 2;
    for (std::size_t i = 0; i < state.numLEDs - 7 - 3 - spacing * 2; i++) {
        state.pixels.at(i) = m_foreground;
    }
    for (int i = state.numLEDs - 7 - 3 - spacing * 2; i < 9 + spacing; i++) {
        state.pixels.at(i) = m_background;
    }
    for (std::size_t i = 9 + spacing; i < state.numLEDs - 3 - spacing; i++) {
        state.pixels.at(i) = m_foreground;
    }
    for (int i = state.numLEDs - 3 - spacing; i < 9 + 7 + spacing * 2; i++) {
        state.pixels.at(i) = m_background;
    }
    for (std::size_t i = 9 + 7 + spacing * 2; i < state.numLEDs; i++) {
        state.pixels.at(i) = m_foreground;
    }
}
}
