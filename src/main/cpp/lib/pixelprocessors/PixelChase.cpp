/*
 * PixelChase.cpp
 *
 *  Created on: Feb 10, 2018
 *      Author: Cole Brinsfield
 */

#include "PixelChase.h"

namespace LightPattern {

PixelChase::PixelChase(PixelStateProcessor* processor, Color background) {
    this->processor = processor;
    this->m_background = background;
}

void PixelChase::Tick(PixelState& state) {
    PixelStateProcessorModulator::Tick(state);

    uint16_t frame = GetFrame(state.frame);

    int k;
    for (std::size_t i = 0; i < state.numLEDs; i++) {
        k = i % 16;
        if ((frame & (1 << k)) >
            0) {  // get the kth bit of frame, if its 1 (OFF)
            state.pixels.at(i) =
                m_background;  // set the pixel in that position to off
        }
    }
}

// 0 is led ON, 1 is led OFF
uint16_t PixelChase::GetFrame(int n) {
    int k = n % 16;
    uint8_t used;
    if (k < 8) {  // n 0-8
        // I want the first n bits of 2nd to be 1, the rest 0
        used = 255 >> (k + 1);
    }
    else {  // 9 to 16, 17 - n is from 8-0
        used = 255 << (16 - k);
    }

    if (n % 32 < 16) {
        return (0xff00 ^
                used);  // return first word all off, second word is animation
    }
    else {
        return (((uint16_t)used << 8) ^ 0x00ff);
    }
}
}
