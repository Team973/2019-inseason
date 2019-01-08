/*
 * RampedOutput.cpp
 *
 *  Created on: Oct 29, 2015
 *      Author: Andrew
 */

#include "lib/helpers/RampedOutput.h"
#include "lib/util/Util.h"
#include <math.h>

namespace frc973 {

RampedOutput::RampedOutput(double rampRate, double initialOutput)
        : m_rampRate(rampRate)
        , m_prevOutput(initialOutput)
        , m_prevTimeMs(0)
        , m_rampFinished(false) {
}

RampedOutput::~RampedOutput() {
}

/**
 * If m_prevTimeMs is 0, that means this is the first call of GetValue...
 * In this special case, just return m_prevOutput (probably 0.0)
 */
double RampedOutput::Update(double input) {
    double timeDiffSec =
        ((double)(GetMsecTime() - m_prevTimeMs)) * Constants::SEC_PER_MSEC;

    double maxOutputDiff = timeDiffSec * m_rampRate;
    double requestedOutputDiff = fabs(input - m_prevOutput);

    if (m_prevTimeMs != 0) {
        m_rampFinished = requestedOutputDiff < maxOutputDiff;

        if (input > m_prevOutput) {
            m_prevOutput += Util::min(maxOutputDiff, requestedOutputDiff);
        }
        else {
            m_prevOutput -= Util::min(maxOutputDiff, requestedOutputDiff);
        }
    }

    m_prevTimeMs = GetMsecTime();

    return m_prevOutput;
}

bool RampedOutput::IsRampFinished(void) {
    return m_rampFinished;
}

void RampedOutput::SetRampRate(double rampRate) {
    m_rampRate = fabs(rampRate);
}

double RampedOutput::GetRampRate(void) {
    return m_rampRate;
}

void RampedOutput::OverridePrevOutput(double prevOutput) {
    m_prevOutput = prevOutput;
}
}
