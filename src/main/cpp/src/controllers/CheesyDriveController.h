/*
 * CheesyDriveController.h
 *
 *  Created on: Jan 15, 2018
 *      Author: Chris
 */

#pragma once

#include "lib/bases/DriveBase.h"
#include "lib/sensors/Limelight.h"

namespace frc973 {

/**
 * Cheesy Drive controller.
 */
class CheesyDriveController : public DriveController {
public:
    /**
     * Construct a CheesyDriveController.
     * @param LimelightHatch The hatch Limelight.
     */
    CheesyDriveController(Limelight *LimelightHatch);
    virtual ~CheesyDriveController();

    /**
     * Calculate motor output given the most recent joystick commands. In this
     * case just return the most recent joystick commands.
     * @param state The DriveStateProvider for handling incoming messages.
     * @param out The DriveControlSignalReceiver for handling outgoing messages.
     */
    void CalcDriveOutput(DriveStateProvider *state,
                         DriveControlSignalReceiver *out);

    /**
     * Checks with the controller to see if we are on target.
     * @return false.
     */
    bool OnTarget() override {
        return false;
    }

    /**
     * Set the joystick values (which in this case will be output).
     * @param throttle Forward/backwards amount.
     * @param turn Left/right amount.
     * @param isQuickTurn Quickturn mode enable/disable.
     * @param isHighGear High gear enable/disable.
     */
    void SetJoysticks(double throttle, double turn, bool isQuickTurn,
                      bool isHighGear);

    /**
     * Start the drive controller.
     * @param out The DriveControlSignalReceiver for handling outgoing messages.
     */
    void Start(DriveControlSignalReceiver *out) override;

    /**
     * Stop the drive controller.
     * @param out The DriveControlSignalReceiver for handling outgoing messages.
     */
    void Stop(DriveControlSignalReceiver *out) override {
        printf("Turning off Cheesy Mode\n");
    }

    /**
     * Gets the left drive output
     * @return The left output
     */
    double GetLeftOutput() {
        return m_leftOutput;
    }

    /**
     * Gets the right drive output
     * @return The right output
     */
    double GetRightOutput() {
        return m_rightOutput;
    }

private:
    Limelight *m_limelightHatch;

    double m_leftOutput;
    double m_rightOutput;
    double m_oldWheel;
    double m_quickStopAccumulator;
    double m_negInertiaAccumulator;

    /*
     * These factor determine how fast the turn traverses the "non linear" sine
     * curve.
     */
    const double kHighWheelNonLinearity = 0.65;
    const double kLowWheelNonLinearity = 0.5;
    const double kHighNegInertiaScalar = 4.0;
    const double kLowNegInertiaThreshold = 0.65;
    const double kLowNegInertiaTurnScalar = 3.5;
    const double kLowNegInertiaCloseScalar = 4.0;
    const double kLowNegInertiaFarScalar = 5.0;
    const double kHighSensitivity = 0.95;
    const double kLowSensitivity = 1.3;
    const double kQuickStopDeadband = 0.2;
    const double kQuickStopWeight = 0.1;
    const double kQuickStopScalar = 5.0;
};
}
