/*
 * Flash.h
 *
 *  Created on: Feb 10, 2018
 *      Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_PIXELPROCESSORS_FLASH_H_
#define SRC_MODULES_PIXELPROCESSORS_FLASH_H_

#include "GreyLightTypes.h"
#include "PixelStateProcessor.h"
#ifndef USING_LED_SIMULATOR
#include "lib/util/Util.h"
#endif
#include <vector>
#include <chrono>

namespace LightPattern {

/**
 * PixelStateProcessor to flash at a set frequency.
 */
class Flash : public PixelStateProcessor {
public:
    /**
     * Construct a Flash Processor.
     * @param first The primary Color to flash.
     * @param second The secondary Color to flash.
     * @param hz The frequency to flash at.
     * @param count The number of times to alternate Colors, omit or -1 for
     * infinite.
     */
    Flash(Color first, Color second, int hz, int count = -1);

    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state) override;

    /**
     * Update the primary and secondary Color.
     * @param first The primary Color to flash.
     * @param second The secondary Color to flash.
     */
    void SetColors(Color first, Color second);

    /**
     * Set the frequency of the flash.
     * @param hz The frequency to flash at.
     */
    void SetFrequency(int hz);

    /**
     * Reset the flash count.
     */
    void Reset() override;

private:
    bool m_color = false;
    Color m_first, m_second;
    int m_hz;
    int m_count;
    int m_loopCount;
    double m_lastTime;
};
}

#endif /* SRC_MODULES_PIXELPROCESSORS_FLASH_H_ */
