/*
 *  LimelightDriveController.h
 *  Created: 4 December 2018
 */

#pragma once

#include "lib/bases/DriveBase.h"
#include "stdio.h"
#include "math.h"
#include "lib/util/Util.h"
#include "lib/sensors/Limelight.h"
#include "src/info/RobotInfo.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "src/subsystems/HatchIntake.h"
namespace frc973 {

class PID;

class LimelightDriveController : public DriveController {
public:
    /**
     * Construct a Limelight Drive controller.
     * @param limelight The limelight.
     */
    LimelightDriveController(Limelight *limelight, bool isCompSkew,
                             ObservablePoofsJoystick *driverJoystick,
                             HatchIntake *hatchIntake);
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

    double CalcScaleGoalAngleComp();
    double CalcTurnComp();
    double CalcThrottleCap();

    /**
     * Checks with the controller to see if we are on target.
     * @return Whether the controller is at/near setpoint.
     */
    bool OnTarget() override {
        return m_onTarget;
    };

    /**
     * Stop the drive controller.
     * @param out The signal receiver for handling outgoing messages.
     */
    void Stop(DriveControlSignalReceiver *out) override {
        printf("Turning off Limelight Drive Mode\n");
    }

    static constexpr double DRIVE_OUTPUT_MULTIPLIER =
        1.0;  // in native units per degree
    static constexpr double DISTANCE_SETPOINT_ROCKET =
        -7.0 + 6.0;  // in inches from target to robot bumper
    static constexpr double DISTANCE_SETPOINT_CARGO_BAY = -8.0;
    static constexpr double PERIOD = 3.0;
    static constexpr double HATCH_VISION_OFFSET =
        -1.0;  // in degrees -1.0, was -2.0 at p-field 0.96 on real field
    static constexpr double CARGO_VISION_OFFSET = 0.0;  // in degrees
    static constexpr double GOAL_ANGLE_COMP_DISTANCE_MIN = 24.0;
    static constexpr double GOAL_ANGLE_COMP_DISTANCE_MAX = 60.0;
    static constexpr double TURN_COMP_DISTANCE_MIN = 6.0;
    static constexpr double TURN_COMP_DISTANCE_MAX = 24.0;
    static constexpr double THROTTLE_CAP_DISTANCE_MIN = 24.0;
    static constexpr double THROTTLE_CAP_DISTANCE_MAX = 60.0;
    static constexpr double THROTTLE_FEED_FORWARD = 0.05;
    static constexpr double THROTTLE_MIN = 0.2;
    static constexpr double THROTTLE_MAX = 0.5;
    static constexpr double GOAL_ANGLE_COMP_KP = 0.03;

private:
    bool m_onTarget;
    double m_leftSetpoint;
    double m_rightSetpoint;
    bool m_isCompensatingSkew;
    HatchIntake *m_hatchIntake;
    ObservablePoofsJoystick *m_driverJoystick;

    double m_throttle;
    double m_turn;
    double m_goalAngleComp;

    Limelight *m_limelight;
    PID *m_turnPid;
    PID *m_throttlePid;
};
}
