/*
 * AssistedCheesyDriveController.cpp
 *
 *  Created on: Jan 15, 2019
 *      Authors: Kyle
 */

#include "src/controllers/AssistedCheesyDriveController.h"

namespace frc973 {

AssistedCheesyDriveController::AssistedCheesyDriveController(
    Limelight *limelight, VisionOffset offset)
        : m_limelight(limelight)
        , m_visionOffset((offset == VisionOffset::Cargo ? CARGO_VISION_OFFSET
                                                        : HATCH_VISION_OFFSET))
        , m_visionTurnPID(
              new PID(0.04, 0.0,
                      0.002))  // without quickturn p = 0.04 // with quickturn
                               // - doesnt work normally p = 0.003
        , m_leftOutput(0.0)
        , m_rightOutput(0.0)
        , m_oldWheel(0.0)
        , m_quickStopAccumulator(0.0)
        , m_negInertiaAccumulator(0.0) {
}

AssistedCheesyDriveController::~AssistedCheesyDriveController() {
}

void AssistedCheesyDriveController::Start(DriveControlSignalReceiver *out) {
    m_limelight->SetCameraVisionCenter();
    m_limelight->SetLightOn();
}

void AssistedCheesyDriveController::CalcDriveOutput(
    DriveStateProvider *state, DriveControlSignalReceiver *out) {
    out->SetDriveOutputVBus(m_leftOutput, m_rightOutput);
    DBStringPrintf(DBStringPos::DB_LINE4, "assch l=%1.2lf r=%1.2lf",
                   m_leftOutput, m_rightOutput);
    DBStringPrintf(DBStringPos::DB_LINE5, "xoff %2.2lf",
                   m_limelight->GetXOffset() - m_visionOffset);
}

void AssistedCheesyDriveController::SetJoysticks(double throttle, double turn,
                                                 bool isQuickTurn,
                                                 bool isHighGear) {
    double sumTurn = turn + m_visionTurnPID->CalcOutputWithError(
                                m_limelight->GetXOffset() - m_visionOffset);

    double negInertia = sumTurn - m_oldWheel;

    if (isQuickTurn) {
        sumTurn = Util::signSquare(sumTurn);
    }
    m_oldWheel = sumTurn;

    double turnNonLinearity;
    if (isHighGear) {
        turnNonLinearity = kHighWheelNonLinearity;
        double denominator = sin(Constants::PI / 2.0 * turnNonLinearity);
        // Apply a sin function that's scaled to make it feel better.
        sumTurn =
            sin(Constants::PI / 2.0 * turnNonLinearity * sumTurn) / denominator;
        sumTurn =
            sin(Constants::PI / 2.0 * turnNonLinearity * sumTurn) / denominator;
    }
    else {
        turnNonLinearity = kLowWheelNonLinearity;
        double denominator = sin(Constants::PI / 2.0 * turnNonLinearity);
        // Apply a sin function that's scaled to make it feel better.
        sumTurn =
            sin(Constants::PI / 2.0 * turnNonLinearity * sumTurn) / denominator;
        sumTurn =
            sin(Constants::PI / 2.0 * turnNonLinearity * sumTurn) / denominator;
        sumTurn =
            sin(Constants::PI / 2.0 * turnNonLinearity * sumTurn) / denominator;
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
        if (sumTurn * negInertia > 0) {
            // If we are moving away from 0.0, aka, trying to get more turn.
            negInertiaScalar = kLowNegInertiaTurnScalar;
        }
        else {
            // Otherwise, we are attempting to go back to 0.0.
            if (fabs(sumTurn) > kLowNegInertiaThreshold) {
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

    sumTurn += m_negInertiaAccumulator;
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
                alpha * Util::limit(sumTurn, 1.0) * kQuickStopScalar;
        }
        overPower = 1.0;
        angularPower = sumTurn;
    }
    else {
        overPower = 0.0;
        angularPower =
            fabs(throttle) * sumTurn * sensitivity - m_quickStopAccumulator;
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
