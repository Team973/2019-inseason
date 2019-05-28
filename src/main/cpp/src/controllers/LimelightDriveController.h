/*
 *  LimelightDriveController.h
 *  Created: 4 December 2018
 */

#pragma once

#include "lib/bases/DriveBase.h"
#include "lib/helpers/PID.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/util/Util.h"
#include "lib/sensors/Limelight.h"

#include "src/info/RobotInfo.h"
#include "src/subsystems/Elevator.h"
#include "src/subsystems/HatchIntake.h"

namespace frc973 {

/**
 * Limelight Drive Controller
 */
class LimelightDriveController : public DriveController {
public:
    /**
     * Construct a LimelightDriveController.
     * @param logger The LogSpreadsheet object.
     * @param limelight The Limelight.
     * @param isCompSkew Whether using skew compensation.
     * @param driverJoystick The driver's controller.
     * @param hatchIntake The HatchIntake subsystem.
     * @param elevator The Elevator subsystem.
     */
    LimelightDriveController(LogSpreadsheet *logger, Limelight *limelight,
                             bool isCompSkew,
                             ObservablePoofsJoystick *driverJoystick,
                             HatchIntake *hatchIntake, Elevator *elevator);
    virtual ~LimelightDriveController();

    /**
     * Start the drive controller.
     * @param out The DriveControlSignalReceiver for handling outgoing messages.
     */
    void Start(DriveControlSignalReceiver *out) override;

    /**
     * Calculate motor output given the most recent limelight updates.
     * @param state The DriveStateProvider for handling incoming messages.
     * @param out The DriveControlSignalReceiver for handling outgoing messages.
     */
    void CalcDriveOutput(DriveStateProvider *state,
                         DriveControlSignalReceiver *out) override;

    /**
     * Calculate skew PID output by multiplying limelight skew value with a
     * distance and "frame" factor based on target x-offset
     */
    double CalcScaleGoalAngleComp();

    /**
     * Calculate turn PID factor based off distance of target
     */
    double CalcTurnComp();

    /**
     * Checks with the controller to see if we are on target.
     * @return Whether the controller is at/near setpoint.
     */
    bool OnTarget() override {
        return m_onTarget;
    };

    /**
     * Stop the drive controller.
     * @param out The DriveControlSignalReceiver for handling outgoing messages.
     */
    void Stop(DriveControlSignalReceiver *out) override {
        printf("Turning off Limelight Drive Mode\n");
    }

    /**
     * Gets throttle PID output for logging
     */
    double GetThrottlePidOut() const;

    /**
     * Gets turn PID output for logging
     */
    double GetTurnPidOut() const;

    /**
     * Gets skew PID output for logging
     */
    double GetGoalAngleComp() const;

private:
    static constexpr double DISTANCE_SETPOINT_ROCKET =
        -2.0; /**< in inches from target to robot bumper */
    static constexpr double DISTANCE_SETPOINT_CARGO_BAY =
        -9.0; /**< in inches from target to robot bumper */
    static constexpr double HATCH_VISION_OFFSET =
        0.32; /**< physical offset of the Limelight to center of target */

    // Range of distances where the compensation factor is applied
    static constexpr double GOAL_ANGLE_COMP_DISTANCE_MIN =
        24.0; /**< Min distance the Goal Angle Comp will affect PID */
    static constexpr double GOAL_ANGLE_COMP_DISTANCE_MAX =
        60.0; /**< Max distance the Goal Angle Comp will affect PID */
    static constexpr double SKEW_COMP_MULTIPLIER_DISTANCE_MIN =
        17.0; /**< Min distance the Skew Comp will affect PID */
    static constexpr double SKEW_COMP_MULTIPLIER_DISTANCE_MAX =
        24.0; /**< Max distance the Skew Comp will affect PID */
    static constexpr double TURN_COMP_DISTANCE_MIN =
        6.0; /**< Min distance the Turn Comp will affect PID */
    static constexpr double TURN_COMP_DISTANCE_MAX =
        24.0; /**< Max distance the Turn Comp will affect PID */

    // Max and min bounds for PID loop outputs
    static constexpr double THROTTLE_MIN =
        -0.6; /**< Min Throttle output value */
    static constexpr double THROTTLE_MAX =
        0.6;                                 /**< Max Throttle output Value */
    static constexpr double SKEW_MIN = -0.2; /**< Min Skew output Value */
    static constexpr double SKEW_MAX = 0.2;  /**< Max Skew output Value */
    static constexpr double TURN_MIN = -0.4; /**< Min Turn output Value */
    static constexpr double TURN_MAX = 0.4;  /**< Max Turn output Value */

    // PID Gains
    static constexpr double GOAL_ANGLE_COMP_KP =
        0.008; /**< P Value for Goal Angle Comp PID */
    static constexpr double TURN_PID_KP = 0.01;  /**< P Value for Turn PID */
    static constexpr double TURN_PID_KI = 0.0;   /**< K Value for Turn PID */
    static constexpr double TURN_PID_KD = 0.001; /**< D Value for Turn PID */
    static constexpr double THROTTLE_PID_KP =
        0.018; /**< P Value for Throttle PID */
    static constexpr double THROTTLE_PID_KI =
        0.0; /**< I Value for Throttle PID */
    static constexpr double THROTTLE_PID_KD =
        0.002; /**< D Value for Throttle PID */

    bool m_onTarget;
    double m_leftSetpoint;
    double m_rightSetpoint;
    bool m_isCompensatingSkew;
    double m_distance;
    HatchIntake *m_hatchIntake;
    Elevator *m_elevator;

    ObservablePoofsJoystick *m_driverJoystick;

    double m_throttle;
    double m_turn;

    Limelight *m_limelight;
    Elevator::RocketScoreMode m_scoreMode;

    double m_throttlePidOut;
    double m_turnPidOut;
    double m_goalAngleComp;

    PID *m_turnPid;
    PID *m_throttlePid;
};
}
