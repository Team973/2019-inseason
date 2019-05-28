/*
 * MedianFilter.cpp
 *
 *  Created on: Jan 21, 2016
 *      Author: Andrew
 */

#include "lib/filters/MedianFilter.h"

namespace frc973 {

MedianFilter::MedianFilter(int buffSize)
        : m_buffSize(buffSize)
        , m_samples(new double[256])  // TODO revert to double[buffSize]
        , m_idx(0)
        , m_last(0.0) {
    for (int i = 0; i < m_buffSize; i++) {
        m_samples[i] = 0.0;
    }
}

MedianFilter::~MedianFilter() {
    delete[] m_samples;
}

double MedianFilter::Update(double in) {
    m_samples[m_idx] = in;
    m_idx = (m_idx + 1) % m_buffSize;

    double sorted[256];  // TODO revert to sorted[buffSize]

    for (int i = 0; i < m_buffSize; i++) {
        sorted[i] = m_samples[i];
    }

    for (int i = 0; i < m_buffSize; i++) {
        for (int j = 0; j < m_buffSize - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                double tmp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = tmp;
            }
        }
    }

    m_last = sorted[m_buffSize / 2];
    return m_last;
}

double MedianFilter::GetLast() {
    return m_last;
}
}
