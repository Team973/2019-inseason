/*
 *  LimelightTrigController.h
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
#include "lib/helpers/XboxJoystickHelper.h"
#include "src/subsystems/HatchIntake.h"
#include "src/subsystems/Elevator.h"
namespace frc973 {

class PID;

/**
 * Limelight Trig Controller
 */
class LimelightTrigController : public DriveController {
public:
    /**
     * Construct a Limelight Drive controller.
     * @param logger LogSpreadsheet object.
     * @param limelight The limelight.
     * @param driverJoystick The driver's controller.
     * @param m_operatorJoystick The operator joystick object.
     * @param hatchIntake The hatch intake subsystem.
     * @param elevator The elevator subsystem.
     */
    LimelightTrigController(LogSpreadsheet *logger, Limelight *limelight,
                            ObservablePoofsJoystick *driverJoystick,
                            ObservableXboxJoystick *m_operatorJoystick,
                            HatchIntake *hatchIntake, Elevator *elevator);
    virtual ~LimelightTrigController();

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
     * Calculates the Turn Comp PID.
     */
    double CalcTurnComp();

    /**
     * Sets the gyro angle to an inputed value.
     * @param angle The angle to change gyro to.
     */
    void SetAngle(double angle);

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

    /**
     * Returns the Throttle PID output.
     * @return the Throttle PID output.
     */
    double GetThrottlePidOut() const;

    /**
     * Returns the Turn PID output.
     * @return the Throttle PID output.
     */
    double GetTurnPidOut() const;

    /**
     * Returns the Skew PID output.
     * @return the Skew PID output.
     */
    double GetSkewPidOut() const;

    /**
     * Updates the robots current target depending on gyro angle and driver
     * input
     */
    double GetTargetDirectionConstant();

private:
    static constexpr double DISTANCE_SETPOINT_ROCKET =
        0.0;  // in inches from target to robot bumper.
    static constexpr double DISTANCE_SETPOINT_CARGO_BAY =
        -5.0;  // in inches from target to robot bumper.
    static constexpr double HATCH_VISION_OFFSET =
        -1.0;  // physical offset of the limelight to center of target.

    // Range of distances where the compensation factor is applied.
    static constexpr double GOAL_ANGLE_COMP_DISTANCE_MIN =
        24.0;  // Min distance the Goal Angle Comp will affect PID.
    static constexpr double GOAL_ANGLE_COMP_DISTANCE_MAX =
        60.0;  // Max distance the Goal Angle Comp will affect PID.
    static constexpr double SKEW_COMP_MULTIPLIER_DISTANCE_MIN =
        17.0;  // Min distance the Skew Comp will affect PID.
    static constexpr double SKEW_COMP_MULTIPLIER_DISTANCE_MAX =
        24.0;  // Max distance the Skew Comp will affect PID.
    static constexpr double TURN_COMP_DISTANCE_MIN =
        6.0;  // Min distance the Turn Comp will affect PID.
    static constexpr double TURN_COMP_DISTANCE_MAX =
        24.0;  // Max distance the Turn Comp will affect PID.

    // Max and min bounds for PID loop outputs.
    static constexpr double THROTTLE_MIN = -0.7;  // Min Throttle output value.
    static constexpr double THROTTLE_MAX = 0.7;   // Max Throttle output Value.
    static constexpr double SKEW_MIN = -0.2;      // Min Skew output Value.
    static constexpr double SKEW_MAX = 0.2;       // Max Skew output Value.
    static constexpr double TURN_MIN = -0.4;      // Min Turn output Value.
    static constexpr double TURN_MAX = 0.4;       // Max Turn output Value.

    // PID Gains.
    static constexpr double GOAL_ANGLE_COMP_KP =
        0.008;  // P Value for Goal Angle Comp PID.
    static constexpr double TURN_PID_KP = 0.012;  // P Value for Turn PID.
    static constexpr double TURN_PID_KI = 0.0;    // K Value for Turn PID.
    static constexpr double TURN_PID_KD = 0.002;  // D Value for Turn PID.
    static constexpr double THROTTLE_PID_KP =
        0.022;                                      // P Value for Throttle PID.
    static constexpr double THROTTLE_PID_KI = 0.0;  // I Value for Throttle PID.
    static constexpr double THROTTLE_PID_KD =
        0.003;  // D Value for Throttle PID.
    static constexpr double THROTTLE_FEED_FORWARD =
        0.05;                                    // F Value for Throttle PID.
    static constexpr double SKEW_PID_KP = 0.01;  // P Value for Skew Comp PID.
    static constexpr double SKEW_PID_KI = 0.0;   // I Value for Skew Comp PID.
    static constexpr double SKEW_PID_KD = 0.0;   // D Value for Skew Comp PID.

    static constexpr double FRONT_CARGO =
        180.0;  // Front Cargo Ship Gyro Value.
    static constexpr double LEFT_CARGO_BAY =
        90.0;  // Left Cargo Bay Gyro Value.
    static constexpr double RIGHT_CARGO_BAY =
        -90.0;  // Right Cargo Bay Gryo Value
    static constexpr double LEFT_FRONT_ROCKET =
        -150.0;  // Left Front Rocket Gyro Value
    static constexpr double RIGHT_FRONT_ROCKET =
        150.0;  // Right Front Rocket Gyro Value
    static constexpr double LEFT_BACK_ROCKET =
        -30.0;  // Left Rear Rocket Gyro Value
    static constexpr double RIGHT_BACK_ROCKET =
        30.0;  // Right Rear Rocket Gyro Value
    static constexpr double HUMAN_LOADING_STATION =
        0.0;  // Human Loading Station Gyro Value

    bool m_onTarget;
    double m_leftSetpoint;
    double m_rightSetpoint;
    HatchIntake *m_hatchIntake;
    Elevator *m_elevator;
    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;

    double m_throttle;
    double m_turn;
    double m_gyroAngle;
    double m_targetConst;

    Limelight *m_limelight;

    double m_throttlePidOut;
    double m_turnPidOut;
    double m_skewPidOut;

    Elevator::RocketScoreMode m_scoreMode;
    HatchIntake::HatchSolenoidState m_puncherState;

    PID *m_turnPid;
    PID *m_throttlePid;
    PID *m_skewPid;
};
}
