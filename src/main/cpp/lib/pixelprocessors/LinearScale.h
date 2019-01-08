/*
 * Speedometer.h
 *
 *  Created on: Mar 30, 2017
 *      Author: Cole
 */
#include "GreyLightTypes.h"
#include "PixelStateProcessor.h"

#ifndef SRC_MODULES_PIXELPROCESSORS_LINEARSCALE_H_
#define SRC_MODULES_PIXELPROCESSORS_LINEARSCALE_H_

namespace LightPattern {

/**
 * PixelStateModulator that overlays a singular bar of a bargraph over a
 * processor.
 */
class LinearScale : public PixelStateProcessorModulator {
public:
    /**
     * Construct a LinearScale Modulator.
     * @param minColor The Color to represent the minimum value.
     * @param maxColor The Color to represent the maximum value.
     * @param minVal The minimum value to be represented by minColor.
     * @param maxVal The maximum value to be represented by m_maxColor.
     * @param modulator The PixelStateProcessor for the line to be overlayed on.
     */
    LinearScale(Color minColor, Color maxColor, double minVal, double maxVal,
                PixelStateProcessor* modulator);

    /**
     * Update the primary and secondary Color.
     * @param state The PixelState.
     */
    void Tick(PixelState& state);

    /**
     * Set the current value to be represented.
     * @param value The current value to be repesented on the linear scale.
     */
    void UpdateValue(double value);

private:
    Color m_minColor;
    Color m_maxColor;
    double m_minVal, m_maxVal, m_currentValue;
};
}

#endif /* SRC_MODULES_PIXELPROCESSORS_LINEARSCALE_H_ */
