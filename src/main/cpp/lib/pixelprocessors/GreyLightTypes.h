/*
 * GreyLightTypes.h
 *
 *  Created on: Feb 5, 2018
 *      Author: Cole Brinsfield
 */

#ifndef SRC_MODULES_GREYLIGHTTYPES_H_
#define SRC_MODULES_GREYLIGHTTYPES_H_
#include <cstdint>
#include <cmath>
#include <vector>

namespace GreyLightType {
/**
 * Data representation of a Color in (r,g,b) format.
 */
struct Color {
    uint8_t r; /**< The red amount (0-255) */
    uint8_t g; /**< The green amount (0-255) */
    uint8_t b; /**< The blue amount (0-255) */

    /**
     * Generate an intermediate Color.
     * @param other The Color to gradient to.
     * @param percentR The Percentage of r channel.
     * @param percentG The Percentage of g channel.
     * @param percentB The Percentage of b channel.
     * @return The Color.
     */
    Color gradientTo(Color other, double percentR, double percentG,
                     double percentB) const {
        return Color{
            uint8_t(r + (other.r - r) * percentR),
            uint8_t(g + (other.g - g) * percentG),
            uint8_t(b + (other.b - b) * percentB),
        };
    }

    /**
     * Generate an intermediate Color.
     * @param other The Color to gradient to.
     * @param percent The Percentage of transition to generate.
     * @return The gradientTo.
     */
    Color gradientTo(Color other, double percent) const {
        return gradientTo(other, percent, percent, percent);
    }
};

/**
 * Collection of parameters for each frame of data.
 */
struct PixelState {
    unsigned int frame;   /**< The current frame (+1 per tick). */
    unsigned int fps;     /**< The framerate. */
    unsigned int numLEDs; /**< The number of LEDs. */
    unsigned int delta;   /**< The time in milliseconds since last tick. */
    std::vector<Color>
        pixels; /**< The array of pixels (this gets displayed). */
};
}
#endif /* SRC_MODULES_GREYLIGHTTYPES_H_ */
