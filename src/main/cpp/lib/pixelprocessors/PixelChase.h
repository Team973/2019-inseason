/*
 * PixelChase.h
 * Creates an "inch-worm" of pixels moving from one side to the other of the
 * strip Created on: Feb 10, 2018 Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_PIXELPROCESSORS_PIXELCHASE_H_
#define SRC_MODULES_PIXELPROCESSORS_PIXELCHASE_H_

#include "GreyLightTypes.h"
#include "PixelStateProcessor.h"
#include <vector>

namespace LightPattern {
/**
 * PixelStateModulator to create an "inch-worm" of pixels moving from one side
 * to the other of the strip.
 */
class PixelChase : public PixelStateProcessorModulator {
public:
    /**
     * Construct a PixelChase Modulator.
     * @param processor The PixelStateProcessor to overlay.
     * @param background The Color to overlay processor on to.
     */
    PixelChase(PixelStateProcessor* processor, Color background);

    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state);

private:
    /**
     * Generates the pattern to use for the "inch-worm".
     * @param n The current Frame number.
     * @return The binary visual representation of "inch-worm".
     */
    uint16_t GetFrame(int n);
    Color m_background;
};
}

#endif /* SRC_MODULES_PIXELPROCESSORS_PIXELCHASE_H_ */
