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
     * @param isCompSkew Boolean on if using skew compensation.
     * @param driverJoystick The driver's controller.
     * @param HatchIntake The hatch intake subsystem.
     */
    LimelightDriveController(LogSpreadsheet *logger, Limelight *limelight,
                             bool isCompSkew,
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

    double GetThrottlePidOut() const;
    double GetTurnPidOut() const;
    double GetGoalAngleComp() const;

    static constexpr double DRIVE_OUTPUT_MULTIPLIER =
        1.0;  // in native units per degree
    static constexpr double DISTANCE_SETPOINT_ROCKET =
        -7.0 + 6.0;  // in inches from target to robot bumper
    static constexpr double DISTANCE_SETPOINT_CARGO_BAY = -8.0;  // prac-sac
    static constexpr double PERIOD = 3.0;
    static constexpr double HATCH_VISION_OFFSET =
        -1.0;  // in degrees -1.0, was -2.0 at p-field 0.96 on real field
    static constexpr double CARGO_VISION_OFFSET = 0.0;  // in degrees
    static constexpr double GOAL_ANGLE_COMP_DISTANCE_MIN = 24.0;
    static constexpr double GOAL_ANGLE_COMP_DISTANCE_MAX = 60.0;
    static constexpr double SKEW_COMP_MULTIPLIER_DISTANCE_MIN = 17.0;
    static constexpr double SKEW_COMP_MULTIPLIER_DISTANCE_MAX = 24.0;
    static constexpr double TURN_COMP_DISTANCE_MIN = 6.0;
    static constexpr double TURN_COMP_DISTANCE_MAX = 24.0;
    static constexpr double THROTTLE_CAP_DISTANCE_MIN = 24.0;
    static constexpr double THROTTLE_CAP_DISTANCE_MAX = 60.0;
    static constexpr double THROTTLE_FEED_FORWARD = 0.05;
    static constexpr double THROTTLE_MIN = -0.7;  // prac-sac
    static constexpr double THROTTLE_MAX = 0.7;   // prac-sac
    static constexpr double GOAL_ANGLE_COMP_KP = 0.02;
    static constexpr double TURN_PID_KP = 0.015;  // prac-sac
    static constexpr double TURN_PID_KI = 0.0;
    static constexpr double TURN_PID_KD = 0.002;
    static constexpr double THROTTLE_PID_KP = 0.023;  // prac-sac
    static constexpr double THROTTLE_PID_KI = 0.0;
    static constexpr double THROTTLE_PID_KD = 0.003;

    /**
     * DBString inputed values for limelight
     */
    void UpdateLimelightDriveDB();
    void CreateLimelightDriveDB();

private:
    bool m_onTarget;
    double m_leftSetpoint;
    double m_rightSetpoint;
    bool m_isCompensatingSkew;
    HatchIntake *m_hatchIntake;
    ObservablePoofsJoystick *m_driverJoystick;

    double m_throttle;
    double m_turn;

    Limelight *m_limelight;

    double m_throttlePidOut;
    double m_turnPidOut;
    double m_goalAngleComp;

    PID *m_turnPid;
    PID *m_throttlePid;

    double m_DBThrottlePIDkP = 0.0;
    double m_DBThrottlePIDkI = 0.0;
    double m_DBThrottlePIDkD = 0.0;
    PID *m_DBThrottlePID;
    double m_DBThrottlePIDOut = 0.0;
    double m_DBTurnPIDkP = 0.0;
    double m_DBTurnPIDkI = 0.0;
    double m_DBTurnPIDkD = 0.0;
    PID *m_DBTurnPID;
    double m_DBTurnPIDOut = 0.0;
    double m_DBGoalAngleCompkP = 0.0;
    double m_DBGoalAngleComp = 0.0;
    double m_DBThrottleFeedForward = 0.0;
    double m_DBHatchVisionOffset = 0.0;
    double m_DBCargoVisionOffset = 0.0;
    double m_DBDistanceSetpointRocket = 0.0;
    double m_DBDistanceSetpointCargoBay = 0.0;
    double m_DBThrottleMin = 0.0;
    double m_DBThrottleMax = 0.0;

    double angle_comp = 0.0;
};
}
