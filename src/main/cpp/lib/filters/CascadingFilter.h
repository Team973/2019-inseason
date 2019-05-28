/*
 * CascadingFilter.h
 *
 *  Created on: Feb 29, 2016
 *      Author: Andrew
 */

#pragma once

#include <vector>

#include "lib/filters/FilterBase.h"

namespace frc973 {

/**
 * A Cascading filter contains a list of filters and applies them
 * sequentially to the signal.
 */
class CascadingFilter : FilterBase {
public:
    /**
     * Construct a CascadingFilter.
     */
    CascadingFilter();
    virtual ~CascadingFilter();

    /**
     * Add a new filter to the sequence.
     * @param newFilt The new filter.
     */
    void PushFilter(FilterBase *newFilt);

    /**
     * Input a new value to check against.
     * @param in The new value.
     */
    double Update(double in) override;

    /**
     * Gets the last value sent to the filter.
     * @return The last value.
     */
    double GetLast() override;

private:
    double m_last;
    std::vector<FilterBase *> m_children;
};
}
