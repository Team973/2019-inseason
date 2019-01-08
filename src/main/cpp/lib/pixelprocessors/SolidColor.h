/*
 * SolidColor.h
 *
 *  Created on: Feb 10, 2018
 *      Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_PIXELPROCESSORS_SOLIDCOLOR_H_
#define SRC_MODULES_PIXELPROCESSORS_SOLIDCOLOR_H_

#include "GreyLightTypes.h"
#include "PixelStateProcessor.h"
#include <vector>

namespace LightPattern {
/**
 * PixelStateProcessor to fill Pixel Data with a solid Color.
 */
class SolidColor : public PixelStateProcessor {
public:
    /**
     * Construct a SolidColor Processor that is Green (default).
     */
    SolidColor();

    /**
     * Construct a SolidColor Processor.
     * @param color The Color to set all pixels to.
     */
    SolidColor(Color color);

    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state) override;

    /**
     * Set the Color.
     * @param color The Color to set all pixels to.
     */
    void SetColor(Color color);

private:
    Color m_currentColor;
};
}
#endif /* SRC_MODULES_PIXELPROCESSORS_SOLIDCOLOR_H_ */
