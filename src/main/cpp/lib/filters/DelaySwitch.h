/*
 * DelaySwitch.h
 *
 *  Created on: Nov 3, 2015
 *      Author: Andrew
 *
 * DelaySwitch filters a binary filter that may be noisy and creates a binary
 * signal that is definitely not noisy but may be a little delayed.
 *
 * Does this by creating a MovingAverageFilter of all previous binary inputs
 * and doesn't change output until the moving average of input is way above or
 * way below 0.5.
 */

#pragma once

#include "lib/filters/MovingAverageFilter.h"

namespace frc973 {

/**
 * A direct-use filter for something that needs to delay between on/off. Note:
 * this class does not inherit from FilterBase because it returns booleans
 * rather than doubles. And no, we are not going to use templates!
 */
class DelaySwitch {
public:
    /**
     * Contstruct a delay switch.
     * @param filteratude Higher numbers of filteratude mean even more delay
     * between on and off... unless the numbers get above 1.0 in which case
     * don't do that! Also don't put negative numbers in otherwise this filter
     * will go back in time to change past points.
     */
    DelaySwitch(double filteratude = 0.9);
    virtual ~DelaySwitch();

    /**
     * Calculate the filtered value given the original datapoint.
     * @param currentValue The current data point that needs to be filtered.
     * @return Result of filtering calculation.
     */
    bool Update(bool currentValue);

    /**
     * Remember the latest value calculated by filtering.
     * @return result of previous filtering calcuation.
     */
    bool GetLast();

private:
    bool m_prevValue;
    MovingAverageFilter m_filter;
};
}
