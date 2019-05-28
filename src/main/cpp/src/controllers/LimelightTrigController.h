/*
 *  LimelightTrigController.h
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
 * Limelight Trig Controller.
 */
class LimelightTrigController : public DriveController {
public:
    /**
     * Construct a LimelightTrigController.
     * @param logger The LogSpreadsheet object.
     * @param limelight The Limelight.
     * @param driverJoystick The driver's ObservablePoofsJoystick.
     * @param operatorJoystick The operator's ObservableXboxJoystick.
     * @param hatchIntake The HatchIntake subsystem.
     * @param elevator The Elevator subsystem.
     */
    LimelightTrigController(LogSpreadsheet *logger, Limelight *limelight,
                            ObservablePoofsJoystick *driverJoystick,
                            ObservableXboxJoystick *operatorJoystick,
                            HatchIntake *hatchIntake, Elevator *elevator);
    virtual ~LimelightTrigController();

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
     * Calculates the Turn Comp PID.
     * @return The calculated Turn Comp PID.
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
     * @param out The DriveControlSignalReceiver for handling outgoing messages.
     */
    void Stop(DriveControlSignalReceiver *out) override {
        printf("Turning off Limelight Drive Mode\n");
    }

    /**
     * Gets the Throttle PID output.
     * @return The Throttle PID output.
     */
    double GetThrottlePidOut() const;

    /**
     * Gets the Turn PID output.
     * @return The Throttle PID output.
     */
    double GetTurnPidOut() const;

    /**
     * Gets the Skew PID output.
     * @return The Skew PID output.
     */
    double GetSkewPidOut() const;

    /**
     * Get the current target direction constant.
     * @return The current target direction constant.
     */
    double GetTargetDirectionConstant();

private:
    static constexpr double DISTANCE_SETPOINT_ROCKET = 0.0;
    static constexpr double DISTANCE_SETPOINT_CARGO_BAY = -5.0;
    static constexpr double HATCH_VISION_OFFSET = -1.0;

    // Range of distances where the compensation factor is applied.
    static constexpr double GOAL_ANGLE_COMP_DISTANCE_MIN = 24.0;
    static constexpr double GOAL_ANGLE_COMP_DISTANCE_MAX = 60.0;
    static constexpr double SKEW_COMP_MULTIPLIER_DISTANCE_MIN = 17.0;
    static constexpr double SKEW_COMP_MULTIPLIER_DISTANCE_MAX = 24.0;
    static constexpr double TURN_COMP_DISTANCE_MIN = 6.0;
    static constexpr double TURN_COMP_DISTANCE_MAX = 24.0;

    // Max and min bounds for PID loop outputs.
    static constexpr double THROTTLE_MIN = -0.7;
    static constexpr double THROTTLE_MAX = 0.7;
    static constexpr double SKEW_MIN = -0.2;
    static constexpr double SKEW_MAX = 0.2;
    static constexpr double TURN_MIN = -0.4;
    static constexpr double TURN_MAX = 0.4;

    // PID Gains.
    static constexpr double GOAL_ANGLE_COMP_KP = 0.008;
    static constexpr double TURN_PID_KP = 0.012;
    static constexpr double TURN_PID_KI = 0.0;
    static constexpr double TURN_PID_KD = 0.002;
    static constexpr double THROTTLE_PID_KP = 0.022;
    static constexpr double THROTTLE_PID_KI = 0.0;
    static constexpr double THROTTLE_PID_KD = 0.003;
    static constexpr double THROTTLE_FEED_FORWARD = 0.05;
    static constexpr double SKEW_PID_KP = 0.01;
    static constexpr double SKEW_PID_KI = 0.0;
    static constexpr double SKEW_PID_KD = 0.0;

    static constexpr double FRONT_CARGO = 180.0;
    static constexpr double LEFT_CARGO_BAY = 90.0;
    static constexpr double RIGHT_CARGO_BAY = -90.0;
    static constexpr double LEFT_FRONT_ROCKET = -150.0;
    static constexpr double RIGHT_FRONT_ROCKET = 150.0;
    static constexpr double LEFT_BACK_ROCKET = -30.0;
    static constexpr double RIGHT_BACK_ROCKET = 30.0;
    static constexpr double HUMAN_LOADING_STATION = 0.0;

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
