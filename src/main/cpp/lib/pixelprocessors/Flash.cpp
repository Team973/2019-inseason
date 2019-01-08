/*
 * Flash.cpp
 *
 *  Created on: Feb 10, 2018
 *      Author: Cole Brinsfield
 */

#include "Flash.h"

using namespace std::chrono;
#include <iostream>
namespace LightPattern {

Flash::Flash(Color first, Color second, int hz, int count)
        : m_first(first), m_second(second), m_hz(hz), m_count(count) {
}

void Flash::SetColors(Color first, Color second) {
    this->m_first = first;
    this->m_second = second;
}
void Flash::Reset() {
    m_loopCount = 0;
    m_lastTime = 0;
}
void Flash::SetFrequency(int hz) {
    this->m_hz = hz;
}

void Flash::Tick(PixelState& state) {
    if (m_count > 0 && m_loopCount > m_count) {
        std::fill(state.pixels.begin(), state.pixels.end(), m_second);
        return;
    }
#ifndef USING_LED_SIMULATOR
    double currentTime = frc973::GetMsecTime();
#else
    double currentTime = 0;
#endif
    // duration_cast<milliseconds>(system_clock::now().time_since_epoch())
    //.count();
    if (currentTime - m_lastTime >= 1.0 / m_hz * 1000) {
        m_color = !m_color;
        m_lastTime = currentTime;
        m_loopCount++;
    }
    if (m_color) {
        std::fill(state.pixels.begin(), state.pixels.end(), m_first);
    }
    else {
        std::fill(state.pixels.begin(), state.pixels.end(), m_second);
    }
}
}
