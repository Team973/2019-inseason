/*
 * TeamNumber.h
 *
 *  Created on: Feb 8, 2018
 *      Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_PIXELPROCESSORS_TEAMNUMBER_H_
#define SRC_MODULES_PIXELPROCESSORS_TEAMNUMBER_H_

#include "GreyLightTypes.h"
#include "PixelStateProcessor.h"

namespace LightPattern {
/**
 * PixelStateProcessor to light up 9 pixels, a gap, 7 pixels, a gap, 3 pixels.
 */
class TeamNumber : public PixelStateProcessor {
public:
    /**
     * Construct a TeamNumber Processor with Default Colors.
     */
    TeamNumber();

    /**
     * Construct a TeamNumber Processor.
     * @param foreground The foreground Color.
     * @param background The background Color.
     */
    TeamNumber(Color foreground, Color background);

    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state) override;

private:
    Color m_foreground;
    Color m_background;
};
}

#endif /* SRC_MODULES_PIXELPROCESSORS_TEAMNUMBER_H_ */
