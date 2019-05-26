/*
 * ReverseModifier.h
 *
 *  Created on: March 11, 2018
 *      Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_PIXELPROCESSORS_REVERSEMODIFIER_H_
#define SRC_MODULES_PIXELPROCESSORS_REVERSEMODIFIER_H_

#include <algorithm>

#include "lib/pixelprocessors/PixelStateProcessor.h"

namespace frc973 {

namespace LightPattern {
/**
 * PixelStateModulator to reverse a PixelStateProcessor.
 */
class ReverseModifier : public PixelStateProcessorModulator {
public:
    /**
     * Construct a ReverseModifier.
     * @param processor The PixelStateProcessor to reverse.
     */
    ReverseModifier(PixelStateProcessor* processor);

    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state) override;
};
}
}

#endif /* SRC_MODULES_PIXELPROCESSORS_REVERSEMODIFIER_H_ */
