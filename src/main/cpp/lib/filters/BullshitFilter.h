/*
 * BullshitFilter.h
 *
 * Bullshit filter looks at a signal and calls bullshit when a point is
 * outside of a valid range.  The behavior when the filter sees a point
 * outside of that range is configurable.
 *
 *  Created on: Feb 29, 2016
 *      Author: andrew
 */

#pragma once

#include "lib/filters/FilterBase.h"

namespace frc973 {

/**
 * A direct-use filter for something that needs to check against absurd values.
 */
class BullshitFilter : public FilterBase {
public:
    /**
     * What do we do for absurdly high values?
     */
    enum MaxBehavior
    {
        noMax,   /**< There is no cap... positive infinity is just gucci. */
        clipMax, /**< If the input is greater than the max, just go with the
                    max. */
        dropMax  /**< If the input is greater than the max, use the previous
                    acceptable value. */
    };

    /**
     * What do we do for absurdly low values?
     */
    enum MinBehavior
    {
        noMin,   /**< There is no cap... negative infinity is just gucci. */
        clipMin, /**< If the input is less than the min, just go with the min.
                  */
        dropMin  /**< If the input is less than the min, use the previous
                    acceptable value. */
    };

    /**
     * Construct a bullshit filter.
     * @param minBehavior Specifies the behavior to use when the value is less
     * than min.
     * @param min The lower limit.
     * @param maxBehavior Specifies the behavior to use when the value is
     * greater than max.
     * @param max The upper limit.
     */
    BullshitFilter(MinBehavior minBehavior, double min, MaxBehavior maxBehavior,
                   double max);
    virtual ~BullshitFilter();

    /**
     * Calculate the filtered value given the original datapoint.
     * @param in The current data point that needs to be filtered.
     * @return Result of filtering calculation.
     */
    double Update(double in) override;

    /**
     * Return the last value sent to the filter.
     * @return The last value.
     */
    double GetLast() override;

private:
    MinBehavior m_minBehavior;
    double m_min;

    MaxBehavior m_maxBehavior;
    double m_max;

    double m_last;
};
}
