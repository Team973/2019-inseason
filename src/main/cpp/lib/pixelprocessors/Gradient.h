/*
 * Gradient.h
 *
 *  Created on: Feb 10, 2018
 *      Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_PIXELPROCESSORS_GRADIENT_H_
#define SRC_MODULES_PIXELPROCESSORS_GRADIENT_H_

#include <vector>
#include "GreyLightTypes.h"
#include "PixelStateProcessor.h"

namespace LightPattern {
/**
 * PixelStateProcessor to generate a Gradient.
 */
class Gradient : public PixelStateProcessor {
public:
    /**
     * Construct a Gradient.
     * @param gradientStart The Color to gradient from.
     * @param gradientEnd The Color to gradient to.
     */
    Gradient(Color gradientStart, Color gradientEnd);

    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state) override;

private:
    Color m_gradientStart;
    Color m_gradientEnd;
};
}

#endif /* SRC_MODULES_PIXELPROCESSORS_GRADIENT_H_ */
