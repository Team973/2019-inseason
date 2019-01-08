/*
 * MovingAverageFilter.cpp
 *
 *  Created on: Oct 18, 2015
 *      Author: Andrew
 */

#include "lib/filters/MovingAverageFilter.h"

namespace frc973 {

MovingAverageFilter::MovingAverageFilter(double weight, double initial)
        : m_weight(weight), m_prevValue(initial) {
}

MovingAverageFilter::~MovingAverageFilter() {
}

double MovingAverageFilter::Update(double currentValue) {
    m_prevValue = (m_weight)*m_prevValue + (1.0 - m_weight) * currentValue;
    return m_prevValue;
}

double MovingAverageFilter::GetLast(void) {
    return m_prevValue;
}
}
