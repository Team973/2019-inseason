/*
 * LengthModifier.h
 *
 *  Created on: March 11, 2018
 *      Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_PIXELPROCESSORS_LENGTHMODIFIER_H_
#define SRC_MODULES_PIXELPROCESSORS_LENGTHMODIFIER_H_

#include "lib/pixelprocessors/PixelStateProcessor.h"

namespace frc973 {

namespace LightPattern {
/**
 * PixelStateProcessorModulator to trick a Processor into believing it's a
 * different length than actual strip.
 */
class LengthModifier : public PixelStateProcessorModulator {
public:
    /**
     * Construct a LengthModifier.
     * @param processor The PixelStateProcessor to modify the length of.
     * @param numLEDs The length the processor should run at.
     */
    LengthModifier(PixelStateProcessor* processor, int numLEDs);

    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state) override;

private:
    int m_numLEDS;
};
}
}

#endif /* SRC_MODULES_PIXELPROCESSORS_LENGTHMODIFIER_H_ */
