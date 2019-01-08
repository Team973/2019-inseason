/*
 * PixelStateProcessor.h
 *
 *  Created on: Feb 5, 2018
 *      Author: Cole Brinsfield
 */
#ifndef SRC_MODULES_PIXELSTATEPROCESSOR_H_
#define SRC_MODULES_PIXELSTATEPROCESSOR_H_

#include <vector>
#include "GreyLightTypes.h"

using namespace GreyLightType;
namespace LightPattern {
/**
 * An object that generates Pixel Data to create patterns and animation
 */
class PixelStateProcessor {
public:
    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    virtual void Tick(PixelState& state) = 0;

    /**
     * Reset the PixelStateProcessor.
     */
    virtual void Reset(){};
    // empty virtual to stop compiler warnings
};

/**
 * An object that manipulates Pixel Data to further extend PixelStateProcessors.
 */
class PixelStateProcessorModulator : public PixelStateProcessor {
public:
    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state) {
        this->processor->Tick(state);
    }

    /**
     * Set the processor to manipulate.
     * @param processor PixelStateProcessor to manipulate.
     */
    void SetProcessor(PixelStateProcessor* processor) {
        this->processor = processor;
    }

    /**
     * Reset the PixelStateModulator
     */
    void Reset() {
        this->processor->Reset();
    }

    PixelStateProcessor* processor; /**< The PixelStateProcessor. */
};
}

#endif /* SRC_MODULES_PIXELSTATEPROCESSOR_H_ */
