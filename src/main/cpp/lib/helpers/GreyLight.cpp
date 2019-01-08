/*
 * GreyLight.cpp
 * Updated version of FRC 114's HellaDees from 2017
 * Handles pixel manipulation then sends it to the APA102 interface to write
 *  Created on: Feb 5, 2018
 *      Author: Cole Brinsfield
 */

#include "GreyLight.h"
#include "lib/pixelprocessors/SolidColor.h"
#include <iostream>

using namespace GreyLightType;

GreyLight::GreyLight(int numLEDs) {
    m_numLEDs = numLEDs;
    m_state = PixelState{};
    m_state.fps = 60;
    m_state.numLEDs = numLEDs;
    m_state.pixels = std::vector<Color>(numLEDs);
    m_processor = new SolidColor({0, 0, 0});  // start with all lights off
#ifndef USING_LED_SIMULATOR
    m_strip = new APA102(numLEDs);
    m_worker = std::thread(&GreyLight::Loop, this);
#endif
}

void GreyLight::Loop() {
    // clock is in seconds
    int lastTick = clock();
#ifndef USING_LED_SIMULATOR
    while (true) {
#endif
        m_stateLock.lock();
        m_state.frame++;
        int now = clock();
        m_state.delta = (now - lastTick) / 1000;
        lastTick = now;
        m_state.numLEDs = m_numLEDs;
        m_processor->Tick(m_state);
#ifndef USING_LED_SIMULATOR

        m_strip->Show(m_state.pixels);
#endif
        m_stateLock.unlock();
        std::this_thread::sleep_for(
            std::chrono::milliseconds(30));  // TO-DO, fps delay math
#ifndef USING_LED_SIMULATOR
    }
#endif
}

PixelState GreyLight::GetState() {
    return m_state;
}

void GreyLight::SetPixelStateProcessor(PixelStateProcessor* processor) {
    m_stateLock.lock();
    this->m_processor = processor;
    this->m_processor->Reset();
    m_stateLock.unlock();
}
