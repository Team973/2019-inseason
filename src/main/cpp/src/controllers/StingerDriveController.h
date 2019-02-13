/*
 * StingerDriveController.h
 *
 *  Created on: Oct 30, 2015
 *      Author: Kyle
 */

#pragma once

#include "lib/bases/DriveBase.h"
#include <stdio.h>

using namespace frc;

namespace frc973 {

/**
 * Stinger Drive controller.
 */
class StingerDriveController : public DriveController {
public:
    /**
     * Construct a Stinger Drive controller.
     */
    StingerDriveController();
    virtual ~StingerDriveController();

    /**
     * Calculate motor output given the most recent joystick commands. In this
     * case just return the most recent joystick commands.
     * @param state The state provider for handling incoming messages.
     * @param out The signal receiver for handling outgoing messages.
     */
    void CalcDriveOutput(DriveStateProvider *state,
                         DriveControlSignalReceiver *out);

    /**
     * Checks with the controller to see if we are on target.
     * @return false, this controller is open-loop.
     */
    bool OnTarget() override {
        return false;
    }

    /**
     * Set the joystick values (which in this case will be output).
     * @param throttle Forward/backwards amount.
     * @param turn Left/right amount.
     */
    void SetJoysticks(double throttle, double turn);

    /**
     * Gets the Stinger Motors output
     * @return Stinger Motors Speed
     */
    double GetStingerMotorOutput();

    /**
     * Start the drive controller.
     * @param out The signal receiver for handling outgoing messages.
     */
    void Start(DriveControlSignalReceiver *out) override {
        printf("Turning on Open Loop Stinger Mode\n");
    }

    /**
     * Stop the drive controller.
     * @param out The signal receiver for handling outgoing messages.
     */
    void Stop(DriveControlSignalReceiver *out) override {
        printf("Turning off Open Loop Stinger Mode\n");
    }

private:
    double m_leftOutput;
    double m_rightOutput;
    double m_stingerOutput;

    double kStingerThrottleScale = 1.0;
    double kStingerTurnScale = 1.0;
};
}
