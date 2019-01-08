/*
 * CenterMirror.h
 *
 *  Created on: Feb 10, 2018
 *      Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_PIXELPROCESSORS_CENTERMIRROR_H_
#define SRC_MODULES_PIXELPROCESSORS_CENTERMIRROR_H_

#include <vector>
#include <algorithm>
#include "GreyLightTypes.h"
#include "PixelStateProcessor.h"

namespace LightPattern {

/**
 * PixelStateModulator to wrap processors around the middle of the strip,
 * rather than the left or right side.
 */
class CenterMirror : public PixelStateProcessorModulator {
public:
    /**
     * Construct a CenterMirror Modulator.
     * @param processor The PixelStateProcessor to modulate.
     * @param inverse Flip if processor runs inside out or outside in.
     */
    CenterMirror(PixelStateProcessor* processor, bool inverse = false);

    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state);

private:
    /**
     * Move data in a Vector to the right half of a Vector.
     * @param state The PixelState for the frame.
     */
    void PushToRightSide(PixelState& state);

    bool m_inverse;
    uint8_t m_previousNumLEDs;
};
}
#endif /* SRC_MODULES_PIXELPROCESSORS_CENTERMIRROR_H_ */
