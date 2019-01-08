/*
 * Debouncer.h  Copied from 254.  Don't tell no one
 *
 * Filters a digital signal... if the incoming signal has been false at any
 * point in the last |period|, return false.  If it has been true for the
 * whole |period|, return true.  Helpful for OnTarget and IsFinished type
 * functions.
 *
 *  Created on: Jan 17, 2016
 *      Author: Andrew
 */

#pragma once

#include "lib/util/Util.h"

namespace frc973 {

/**
 * A direct-use filter for something that needs this to debounce values. This
 * filter sets a timer to "debounce" noisy signals and removes many
 * false-positives.
 */
class Debouncer {
public:
    /**
     * Construct a Debouncer object with the given period in seconds.
     * @param period Time in seconds to check for falses.
     */
    Debouncer(double period) {
        m_timeStart = 0.0;
        m_period = period;
        m_first = false;
    }
    virtual ~Debouncer() {
    }

    /**
     * Calculate the filtered value given the original datapoint.
     * @param val The current data point that needs to be filtered.
     * @return Result of filtering calculation.
     */
    bool Update(bool val) {
        if (m_first) {
            m_first = false;
            m_timeStart = GetSecTime();
        }
        if (!val) {
            m_timeStart = GetSecTime();
        }
        return (GetSecTime() - m_timeStart) > m_period;
    }

private:
    double m_timeStart;
    double m_period;
    bool m_first;
};
}
