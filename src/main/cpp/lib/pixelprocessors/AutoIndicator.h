/*
 * AutoIndicator.h
 *
 *  Created on: Feb 10, 2018
 *      Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_PIXELPROCESSORS_AUTOINDICATOR_H_
#define SRC_MODULES_PIXELPROCESSORS_AUTOINDICATOR_H_

#include "GreyLightTypes.h"
#include "PixelStateProcessor.h"
#include <vector>
#include <string>

namespace LightPattern {
static constexpr Color LEFT_GREEN = {0, 255, 0};
static constexpr Color RIGHT_WHITE = {255, 255, 255};
static constexpr Color NO_MESSAGE_BLUE = {0, 0, 255};
static constexpr Color DEFAULT_FILL_RED = {255, 0, 0};

/**
 * PixelStateProcessor to visualize Autonomous info.
 */
class AutoIndicator : public PixelStateProcessor {
public:
    /**
     * Construct an AutoIndicator Processor.
     */
    AutoIndicator();

    /**
     * Generate a new frame of LED Data.
     * @param state The PixelState for the frame.
     */
    void Tick(PixelState& state) override;

    /**
     * Update recieved info for visualization.
     * @param gameDataStr The game data recieved from FMS.
     */
    void SetData(std::string gameDataStr);

private:
    std::string m_gameData;
};
}
#endif /* SRC_MODULES_PIXELPROCESSORS_AUTOINDICATOR_H_ */
