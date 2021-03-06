/*
 * Wave.h
 *
 *  Created on: Feb 8, 2018
 *      Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_PIXELPROCESSORS_WAVE_H_
#define SRC_MODULES_PIXELPROCESSORS_WAVE_H_

#include "lib/pixelprocessors/PixelStateProcessor.h"

namespace frc973 {

namespace LightPattern {
/**
 * PixelStateProcessor to display a cosine wave of color.
 */
class Wave : public PixelStateProcessor {
public:
    /**
     * Construct a Wave Processor.
     * @param background The background Color to use.
     * @param foreground The foreground Color to use.
     * @param period The period of the wave.
     */
    Wave(Color background, Color foreground, int period);

    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state) override;

private:
    int m_period;
    Color m_background, m_foreground;
};
}
}

#endif /* SRC_MODULES_PIXELPROCESSORS_WAVE_H_ */
