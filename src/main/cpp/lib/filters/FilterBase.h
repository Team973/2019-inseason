/*
 * FilterBase.h
 *
 *  Created on: Feb 23, 2016
 *      Author: Andrew
 */

#pragma once

namespace frc973 {

/**
 * Interface for a filter. If all filters implement this interface, they can be
 * passed around generically (like in the cascading filter filter).
 */
class FilterBase {
public:
    /**
     * Construct a filter.
     */
    FilterBase();
    virtual ~FilterBase();

    /**
     * Calculate the filtered value given the original datapoint.
     * @param input The current data point that needs to be filtered.
     * @return Result of filtering calculation.
     */
    virtual double Update(double input) = 0;

    /**
     * Return the last value sent to the filter.
     * @return The last value.
     */
    virtual double GetLast() = 0;
};
}
