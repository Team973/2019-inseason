/*
 * Gradient.cpp
 *
 *  Created on: Feb 10, 2018
 *      Author: Cole Brinsfield
 */

#include "Gradient.h"

namespace LightPattern {

Gradient::Gradient(Color m_gradientStart, Color m_gradientEnd)
        : m_gradientStart(m_gradientStart), m_gradientEnd(m_gradientEnd) {
}

void Gradient::Tick(PixelState& state) {
    for (std::size_t i = 0; i < state.numLEDs / 2; i++) {
        double percentage = double(i) / (state.numLEDs / 2);
        Color gradientIndex = {
            uint8_t(m_gradientStart.r +
                    percentage * (m_gradientEnd.r - m_gradientStart.r)),
            uint8_t(m_gradientStart.g +
                    percentage * (m_gradientEnd.g - m_gradientStart.g)),
            uint8_t(m_gradientStart.b +
                    percentage * (m_gradientEnd.b - m_gradientStart.b))};
        state.pixels.at(i) = gradientIndex;
        // std::cout<<"I: "<<i<<" PERCENTANGE 1/2: "<<percentage<<std::endl;
    }
    for (std::size_t i = state.numLEDs / 2; i < state.numLEDs; i++) {
        double percentage = double(i - state.numLEDs / 2) / (state.numLEDs / 2);
        Color gradientIndex = {
            uint8_t(m_gradientEnd.r +
                    percentage * (m_gradientStart.r - m_gradientEnd.r)),
            uint8_t(m_gradientEnd.g +
                    percentage * (m_gradientStart.g - m_gradientEnd.g)),
            uint8_t(m_gradientEnd.b +
                    percentage * (m_gradientStart.b - m_gradientEnd.b))};
        state.pixels.at(i) = gradientIndex;
        // std::cout<<"I: "<<i<<" PERCENTANGE 2/2: "<<percentage<<std::endl;
    }
}
}
