/*
 * LoopModulator.h
 *
 *  Created on: Feb 10, 2018
 *      Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_PIXELPROCESSORS_LOOPMODULATOR_H_
#define SRC_MODULES_PIXELPROCESSORS_LOOPMODULATOR_H_

#include <vector>

#include "GreyLightTypes.h"
#include "PixelStateProcessor.h"

namespace LightPattern {
/**
 * PixelStateModulator to create a "scrolling" effect from a
 * PixelStateProcessor.
 */
class LoopModulator : public PixelStateProcessorModulator {
public:
    /**
     * Generate a new LoopModulator.
     * @param processor The PixelStateProcessor to loop.
     */
    LoopModulator(PixelStateProcessor* processor);

    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state);

private:
    /**
     * Shift Pixels forward.
     * @param pixels The Pixels to move.
     * @param amount How far you want to shift the Pixels.
     * @param numLEDs The number of LEDs in the strip.
     */
    void Rotate(std::vector<Color>& pixels, int amount, int numLEDs);
};
}

#endif /* SRC_MODULES_PIXELPROCESSORS_LOOPMODULATOR_H_ */
