/*
 *  LimelightDriveController.h
 *  Created: 4 December 2018
 */

#pragma once

#include "lib/bases/DriveBase.h"
#include "stdio.h"
#include "lib/util/Util.h"
#include "lib/sensors/Limelight.h"

namespace frc973 {

class PID;

class LimelightDriveController : public DriveController {
public:
    /**
     * Construct a Limelight Drive controller.
     * @param limelight The limelight.
     */
    LimelightDriveController(Limelight *limelight);
    virtual ~LimelightDriveController();

    /**
     * Start the drive controller.
     * @param out The signal receiver for handling outgoing messages.
     */
    void Start(DriveControlSignalReceiver *out) override;

    /**
     * Calculate motor output given the most recent limelight updates.
     * @param state The state provider for handling incoming messages.
     * @param out The signal receiver for handling outgoing messages.
     */
    void CalcDriveOutput(DriveStateProvider *state,
                         DriveControlSignalReceiver *out) override;

    /**
     * Checks with the controller to see if we are on target.
     * @return Whether the controller is at/near setpoint.
     */
    bool OnTarget() override {
        return m_onTarget;
    };

    /**
     * Set the joystick values (which in this case will be output).
     * @param throttle Forward/backwards amount.
     * @param turn Left/right amount.
     */
    void SetJoysticks(double throttle, double turn);

    static constexpr double DRIVE_OUTPUT_MULTIPLIER =
        500.0;  // in native units per degree

private:
    bool m_onTarget;
    double m_leftSetpoint;
    double m_rightSetpoint;

    double m_throttle;
    double m_turn;

    Limelight *m_limelight;
    PID *m_pid;
};
}
