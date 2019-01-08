/*
 * MovingAverageFilter.h
 *
 *  Created on: Oct 18, 2015
 *      Author: Andrew
 *
 * Simple data filter.  Given a value m between 0.0 and 1.0, return the
 * current datapoint times (1 - m) plus the previous datapoint times m.
 */

#pragma once

#include "lib/filters/FilterBase.h"

namespace frc973 {

/**
 * A direct-use filter for something that needs to filter moving averages.
 */
class MovingAverageFilter : public FilterBase {
public:
    /**
     * Create a data filter by the moving average method.
     * @param weight Weight of the previous value when determining the filtered
     * value.
     * @param initial Value to consider as the previous value.
     */
    MovingAverageFilter(double weight, double initial = 0.0);
    virtual ~MovingAverageFilter();

    /**
     * Calculate the filtered value given the original datapoint.
     * @param input The current data point that needs to be filtered.
     * @return Result of filtering calculation.
     */
    double Update(double input);

    /**
     * Return the last value sent to the filter.
     * @return The last value.
     */
    double GetLast();

private:
    double m_weight;
    double m_prevValue;
};
}
