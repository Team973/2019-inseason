/*
 * CascadingFilter.cpp
 *
 *  Created on: Feb 29, 2016
 *      Author: andrew
 */

#include "lib/filters/CascadingFilter.h"

namespace frc973 {

CascadingFilter::CascadingFilter() : m_last(0.0) {
}

CascadingFilter::~CascadingFilter() {
}

void CascadingFilter::PushFilter(FilterBase *child) {
    m_children.push_back(child);
}

double CascadingFilter::Update(double in) {
    m_last = in;

    for (FilterBase *it : m_children) {
        m_last = it->Update(m_last);
    }

    return m_last;
}

double CascadingFilter::GetLast() {
    return m_last;
}
}
