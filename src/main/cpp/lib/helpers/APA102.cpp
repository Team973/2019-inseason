/*
 * APA102.cpp
 * Interface to control APA102 LEDs from the roboRio
 * Based on code written by Adafruit and FRC team 1736
 *  Created on: Feb 19, 2018
 *      Author: Cole Brinsfield
 */

#include "APA102.h"

namespace frc973 {

using namespace GreyLightType;

APA102::APA102(int numLEDs, frc::SPI::Port port) : m_numLEDs(numLEDs) {
    int numBytesForStrip =
        4 * m_numLEDs + 4 + 4;  // 4*numLeds+startFrame length + endFrame length
    m_endFrameOffset = 4 * m_numLEDs + 4;  // 4*numLEDs+startFrame length;
    m_ledBuffer = std::vector<uint8_t>(numBytesForStrip);
    /*
     * One frame of led data consists of four 0x00 bytes, sequential 0xFF, R, G,
     * B for each LED, and then 4 0xFF bytes
     */
    for (int i = 0; i < 4; i++) {
        m_ledBuffer.at(i) = START_FRAME[i];
    }
    for (int i = 0; i < 4; i++) {
        m_ledBuffer.at(i + m_endFrameOffset) = END_FRAME[i];
    }
    m_spi = new frc::SPI(port);
    m_spi->SetMSBFirst();
    m_spi->SetClockActiveLow();
    m_spi->SetClockRate(SPI_CLK_RATE);
    m_spi->SetSampleDataOnTrailingEdge();
}

void APA102::Show(std::vector<Color> pixels) {
    std::reverse(pixels.begin(),
                 pixels.end());  // the strip on the bot is wired
                                 // with 0 on the rightmost side
    int CHUNK_SIZE = 127;        // max TX size
    for (int i = 0; i < m_numLEDs; i++) {
        m_ledBuffer.at(i * BYTES_PER_LED + LED_ZERO_OFFSET + GLOBAL_OFFSET) =
            0xFF;
        m_ledBuffer.at(i * BYTES_PER_LED + LED_ZERO_OFFSET + RED_OFFSET) =
            pixels.at(i).r;
        m_ledBuffer.at(i * BYTES_PER_LED + LED_ZERO_OFFSET + GREEN_OFFSET) =
            pixels.at(i).g;
        m_ledBuffer.at(i * BYTES_PER_LED + LED_ZERO_OFFSET + BLUE_OFFSET) =
            pixels.at(i).b;
    }
    int ledBufferLength = m_ledBuffer.size();
    for (int offset = 0; offset < ledBufferLength; offset += CHUNK_SIZE) {
        int startIndex = offset;
        int endIndex = std::min(offset + CHUNK_SIZE, ledBufferLength);
        int size = endIndex - startIndex;
        uint8_t txArray[512];  // TODO revert to txArray[size]
        for (int i = 0; i < size; i++) {
            txArray[i] = m_ledBuffer.at(startIndex + i);
        }
        m_spi->Write(txArray, size);
    }
}
}
