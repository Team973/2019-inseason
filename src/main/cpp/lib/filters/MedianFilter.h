/*
 * MedianFilter.h
 *
 *  Created on: Jan 17, 2016
 *      Author: Andrew
 */

#pragma once

#include "lib/filters/FilterBase.h"

namespace frc973 {

/**
 * A direct-use filter for something that needs to filter median values.
 */
class MedianFilter : public FilterBase {
public:
    /**
     * Construct a median filter.
     * @param buffSize The buffer size.
     */
    MedianFilter(int buffSize = 5);
    virtual ~MedianFilter();

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
    int m_buffSize;
    double *m_samples;
    int m_idx;
    double m_last;
};
}
