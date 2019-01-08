/*
 * SolidColor.cpp
 *
 *  Created on: Feb 10, 2018
 *      Author: Cole Brinsfield
 */

#include "SolidColor.h"
#include <iostream>
namespace LightPattern {
SolidColor::SolidColor() {
    SolidColor(Color{0, 255, 0});
}
SolidColor::SolidColor(Color color) {
    m_currentColor = color;
}
void SolidColor::SetColor(Color color) {
    m_currentColor = color;
}

void SolidColor::Tick(PixelState& state) {
    std::fill(state.pixels.begin(), state.pixels.begin() + state.numLEDs,
              m_currentColor);
}
}
