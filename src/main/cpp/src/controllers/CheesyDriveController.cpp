/*
 * CheesyDriveController.cpp
 *
 *  Created on: Jan 15, 2018
 *      Authors: Oliver, Chris
 */

#include "src/controllers/CheesyDriveController.h"
#include "lib/util/Util.h"
#include <stdio.h>
#include "lib/util/WrapDash.h"
#include "src/info/RobotInfo.h"

using namespace frc;

namespace frc973 {

CheesyDriveController::CheesyDriveController()
        : m_leftOutput(0.0)
        , m_rightOutput(0.0)
        , m_oldWheel(0.0)
        , m_quickStopAccumulator(0.0)
        , m_negInertiaAccumulator(0.0) {
}

CheesyDriveController::~CheesyDriveController() {
}

void CheesyDriveController::CalcDriveOutput(DriveStateProvider *state,
                                            DriveControlSignalReceiver *out) {
    out->SetDriveOutputVBus(m_leftOutput, m_rightOutput);
    DBStringPrintf(DBStringPos::DB_LINE4, "cheesy l=%1.2lf r=%1.2lf",
                   m_leftOutput, m_rightOutput);
    // printf("cheesy l=%1.2lf r=%1.2lf\n", m_leftOutput, m_rightOutput);
}

void CheesyDriveController::SetJoysticks(double throttle, double turn,
                                         bool isQuickTurn, bool isHighGear) {
    double negInertia = turn - m_oldWheel;
    if (isQuickTurn) {
        turn = Util::signSquare(turn);
    }
    m_oldWheel = turn;

    double turnNonLinearity;
    if (isHighGear) {
        turnNonLinearity = kHighWheelNonLinearity;
        double denominator = sin(Constants::PI / 2.0 * turnNonLinearity);
        // Apply a sin function that's scaled to make it feel better.
        turn = sin(Constants::PI / 2.0 * turnNonLinearity * turn) / denominator;
        turn = sin(Constants::PI / 2.0 * turnNonLinearity * turn) / denominator;
    }
    else {
        turnNonLinearity = kLowWheelNonLinearity;
        double denominator = sin(Constants::PI / 2.0 * turnNonLinearity);
        // Apply a sin function that's scaled to make it feel better.
        turn = sin(Constants::PI / 2.0 * turnNonLinearity * turn) / denominator;
        turn = sin(Constants::PI / 2.0 * turnNonLinearity * turn) / denominator;
        turn = sin(Constants::PI / 2.0 * turnNonLinearity * turn) / denominator;
    }

    double leftPwm, rightPwm, overPower;
    double sensitivity;

    double angularPower;
    double linearPower;

    // Negative inertia!
    double negInertiaScalar;
    if (isHighGear) {
        negInertiaScalar = kHighNegInertiaScalar;
        sensitivity = kHighSensitivity;
    }
    else {
        if (turn * negInertia > 0) {
            // If we are moving away from 0.0, aka, trying to get more turn.
            negInertiaScalar = kLowNegInertiaTurnScalar;
        }
        else {
            // Otherwise, we are attempting to go back to 0.0.
            if (fabs(turn) > kLowNegInertiaThreshold) {
                negInertiaScalar = kLowNegInertiaFarScalar;
            }
            else {
                negInertiaScalar = kLowNegInertiaCloseScalar;
            }
        }
        sensitivity = kLowSensitivity;
    }
    double negInertiaPower = negInertia * negInertiaScalar;
    m_negInertiaAccumulator += negInertiaPower;

    turn += m_negInertiaAccumulator;
    if (m_negInertiaAccumulator > 1) {
        m_negInertiaAccumulator -= 1;
    }
    else if (m_negInertiaAccumulator < -1) {
        m_negInertiaAccumulator += 1;
    }
    else {
        m_negInertiaAccumulator = 0;
    }
    linearPower = -throttle;

    // Quickturn!
    if (isQuickTurn) {
        if (fabs(linearPower) < kQuickStopDeadband) {
            double alpha = kQuickStopWeight;
            m_quickStopAccumulator =
                (1 - alpha) * m_quickStopAccumulator +
                alpha * Util::limit(turn, 1.0) * kQuickStopScalar;
        }
        overPower = 1.0;
        angularPower = turn;
    }
    else {
        overPower = 0.0;
        angularPower =
            fabs(throttle) * turn * sensitivity - m_quickStopAccumulator;
        if (m_quickStopAccumulator > 1) {
            m_quickStopAccumulator -= 1;
        }
        else if (m_quickStopAccumulator < -1) {
            m_quickStopAccumulator += 1;
        }
        else {
            m_quickStopAccumulator = 0.0;
        }
    }

    rightPwm = leftPwm = linearPower;
    leftPwm += angularPower;
    rightPwm -= angularPower;

    if (leftPwm > 1.0) {
        rightPwm -= overPower * (leftPwm - 1.0);
        leftPwm = 1.0;
    }
    else if (rightPwm > 1.0) {
        leftPwm -= overPower * (rightPwm - 1.0);
        rightPwm = 1.0;
    }
    else if (leftPwm < -1.0) {
        rightPwm += overPower * (-1.0 - leftPwm);
        leftPwm = -1.0;
    }
    else if (rightPwm < -1.0) {
        leftPwm += overPower * (-1.0 - rightPwm);
        rightPwm = -1.0;
    }

    m_leftOutput = -leftPwm;
    m_rightOutput = -rightPwm;
}
}
