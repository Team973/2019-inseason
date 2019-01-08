/*
 * CascadingFilter.h
 *
 * A Cascading filter contains a list of filters and applies them sequentially
 * to the signal.
 *
 *  Created on: Feb 29, 2016
 *      Author: andrew
 */

#pragma once

#include "lib/filters/FilterBase.h"
#include <vector>

namespace frc973 {

/**
 * A direct-use filter for something that needs to use multiple filters in a
 * sequence.
 */
class CascadingFilter : FilterBase {
public:
    /**
     * Construct a cascading filter.
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
     * Return the last value sent to the filter.
     * @return The last value.
     */
    double GetLast() override;

private:
    double m_last;
    std::vector<FilterBase *> m_children;
};
}
