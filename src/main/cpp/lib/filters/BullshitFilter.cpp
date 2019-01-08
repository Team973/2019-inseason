/*
 * BullshitFilter.cpp
 *
 *  Created on: Feb 29, 2016
 *      Author: andrew
 */

#include "lib/filters/BullshitFilter.h"

namespace frc973 {

BullshitFilter::BullshitFilter(MinBehavior minBehavior, double min,
                               MaxBehavior maxBehavior, double max)
        : m_minBehavior(minBehavior)
        , m_min(min)
        , m_maxBehavior(maxBehavior)
        , m_max(max)
        , m_last(0.0) {
}

BullshitFilter::~BullshitFilter() {
}

double BullshitFilter::Update(double in) {
    if (in > m_max) {
        switch (m_maxBehavior) {
            case MaxBehavior::noMax:
                break;
            case MaxBehavior::clipMax:
                in = m_max;
                break;
            case MaxBehavior::dropMax:
                in = m_last;
                break;
        }
    }

    if (in < m_min) {
        switch (m_minBehavior) {
            case MinBehavior::noMin:
                break;
            case MinBehavior::clipMin:
                in = m_min;
                break;
            case MinBehavior::dropMin:
                in = m_last;
                break;
        }
    }

    return m_last;
}

double BullshitFilter::GetLast() {
    return m_last;
}
}
