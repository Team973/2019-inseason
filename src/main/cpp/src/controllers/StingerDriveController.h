/*
 * StingerDriveController.h
 *
 *  Created on: Jan 15, 2018
 *      Author: Chris
 */

#pragma once

#include <stdio.h>
#include "lib/bases/DriveBase.h"
#include "src/controllers/CheesyDriveController.h"

using namespace frc;

namespace frc973 {

/**
 * Stinger Drive controller.
 */
class StingerDriveController : public CheesyDriveController {
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
     * @param out The signal receiver for handling outgoing messages.
     */
    void Start(DriveControlSignalReceiver *out) override {
        printf("Turning on Stinger Mode\n");
    }

    /**
     * Stop the drive controller.
     * @param out The signal receiver for handling outgoing messages.
     */
    void Stop(DriveControlSignalReceiver *out) override {
        printf("Turning off Stinger Mode\n");
    }

private:
    double m_leftOutput;
    double m_rightOutput;
    double m_oldWheel;
    double m_quickStopAccumulator;
    double m_negInertiaAccumulator;
};
}
