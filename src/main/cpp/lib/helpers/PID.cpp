#include "frc/WPILib.h"
#include "lib/helpers/PID.h"
#include "lib/util/Util.h"

namespace frc973 {

PID::PID(double Kp, double Ki, double Kd, uint32_t flags) {
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;

    m_target = 0;
    m_min = -1;
    m_max = 1;

    m_timeLastUpdateSec = 0.0;
    m_prevPos = NAN;
    m_prevErr = NAN;
    m_integral = 0;
    m_icap = 0;

    m_flags = flags;
    m_lastOutput = 0.0;
}

PID::PID(double gains[3], uint32_t flags) {
    m_Kp = gains[0];
    m_Ki = gains[1];
    m_Kd = gains[2];

    m_target = 0;
    m_min = -1;
    m_max = 1;

    m_timeLastUpdateSec = 0.0;
    m_prevPos = NAN;
    m_prevErr = NAN;
    m_integral = 0;
    m_icap = 0;

    m_flags = flags;
    m_lastOutput = 0.0;
}

void PID::SetGains(double Kp, double Ki, double Kd) {
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;
}

void PID::SetGains(double gains[3]) {
    m_Kp = gains[0];
    m_Ki = gains[1];
    m_Kd = gains[2];
}

void PID::SetTarget(double target) {
    m_target = target;
}

double PID::GetTarget() {
    return m_target;
}

void PID::Reset(double currPosition) {
    m_integral = 0.0;
    m_prevPos = currPosition;
    m_lastOutput = 0.0;
    m_timeLastUpdateSec = 0.0;
}

void PID::SetICap(double icap) {
    m_icap = icap;
}

void PID::SetBounds(double min, double max) {
    m_min = min;
    m_max = max;
}

double PID::CalcOutput(double actual) {
    return CalcOutput(actual, GetMsecTime());
}

double PID::CalcOutput(double actual, uint32_t time) {
    double error = m_target - actual;
    double derivative = 0;
    double output;

    /**
     * m_timeLastUpdate will be zero if this is the first call ever or first
     * call since a reset.  In this case there aren't enough samples to
     * integrate or differentiate.
     */
    if (m_timeLastUpdateSec != 0.0) {
        double deltaTimeSec = GetSecTime() - m_timeLastUpdateSec;

        m_integral += error * deltaTimeSec;

        if (m_prevPos != NAN) {
            derivative = (actual - m_prevPos) / deltaTimeSec;
        }
    }
    m_timeLastUpdateSec = GetSecTime();
    m_prevPos = actual;

    output = m_Kp * error + Util::bound(m_Ki * m_integral, -m_icap, m_icap) +
             m_Kd * derivative;

    if (m_flags & PID_SPEED_CTRL) {
        output += m_lastOutput;
    }

    output = Util::bound(output, m_min, m_max);
    m_lastOutput = output;

    return output;
}

double PID::CalcOutputWithError(double error) {
    return CalcOutputWithError(error, GetMsecTime());
}

double PID::CalcOutputWithError(double error, uint32_t time) {
    double derivative = 0;
    double output;

    /**
     * m_timeLastUpdate will be zero if this is the first call ever or first
     * call since a reset.  In this case there aren't enough samples to
     * integrate or differentiate.
     */
    if (m_timeLastUpdateSec != 0.0) {
        double deltaTimeSec = GetSecTime() - m_timeLastUpdateSec;

        m_integral += error * deltaTimeSec;

        if (m_prevPos != NAN) {
            derivative = (error - m_prevErr) / deltaTimeSec;
        }
    }
    m_timeLastUpdateSec = GetSecTime();
    m_prevErr = error;

    output = m_Kp * error + Util::bound(m_Ki * m_integral, -m_icap, m_icap) +
             m_Kd * derivative;

    if (m_flags & PID_SPEED_CTRL) {
        output += m_lastOutput;
    }

    output = Util::bound(output, m_min, m_max);
    m_lastOutput = output;

    return output;
}

double PID::GetPrevOutput() {
    return m_lastOutput;
}

void PID::SetPrevOutput(double prev) {
    m_lastOutput = Util::bound(prev, m_min, m_max);
}
}
