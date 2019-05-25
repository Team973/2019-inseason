/*
 * APA102.h
 *  Created on: Feb 5, 2018
 *      Author: Cole Brinsfield
 */
#ifndef SRC_MODULES_APA102_H_
#define SRC_MODULES_APA102_H_

#include <algorithm>
#include <cmath>
#include <vector>

#include "frc/WPILib.h"
#include "lib/pixelprocessors/GreyLightTypes.h"

using namespace GreyLightType;

/**
 * Interface for an LED system.
 */
class APA102 {
public:
    /**
     * Construct an APA102 system.
     * @param numLEDs The number of LEDs to control.
     * @param port The SPI port the APA102 strip is wired to.
     */
    APA102(int numLEDs, frc::SPI::Port port = frc::SPI::Port::kMXP);

    /**
     * Update APA102 strip with new data.
     * @param pixels The vector of pixels to send to the APA102 strip.
     */
    void Show(std::vector<Color> pixels);

private:
    const uint8_t START_FRAME[4] = {0x00, 0x00, 0x00, 0x00};
    const uint8_t END_FRAME[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    const int GLOBAL_OFFSET = 0;
    const int BLUE_OFFSET = 1;
    const int RED_OFFSET = 3;
    const int GREEN_OFFSET = 2;
    const int BYTES_PER_LED = 4;
    const int LED_ZERO_OFFSET = 4;  // length of startFrame
    const int SPI_CLK_RATE = 13000000;
    int m_numLEDs;
    int m_endFrameOffset;  // will be dependent on num Leds
    frc::SPI* m_spi;
    std::vector<uint8_t> m_ledBuffer;
};

#endif
