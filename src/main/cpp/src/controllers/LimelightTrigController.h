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
namespace frc973 {

class PID;

class LimelightTrigController : public DriveController {
public:
    /**
     * Construct a Limelight Drive controller.
     * @param limelight The limelight.
     * @param isCompSkew Boolean on if using skew compensation.
     * @param driverJoystick The driver's controller.
     * @param HatchIntake The hatch intake subsystem.
     */
    LimelightTrigController(LogSpreadsheet *logger, Limelight *limelight,
                            ObservablePoofsJoystick *driverJoystick,
                            ObservableXboxJoystick *m_operatorJoystick,
                            HatchIntake *hatchIntake);
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
     * @param out The signal receiver for handling outgoing messages.
     */
    void Stop(DriveControlSignalReceiver *out) override {
        printf("Turning off Limelight Drive Mode\n");
    }
    double GetThrottlePidOut() const;
    double GetTurnPidOut() const;
    double GetSkewPidOut() const;

    double GetTargetDirectionConstant();

    static constexpr double DISTANCE_SETPOINT_ROCKET =
        0.0;  // in inches from target to robot bumper
    static constexpr double DISTANCE_SETPOINT_CARGO_BAY = -5.0;  // prac-sac
    static constexpr double HATCH_VISION_OFFSET =
        -1.0;  // in degrees -1.0, was -2.0 at p-field 0.96 on real field
    static constexpr double CARGO_VISION_OFFSET = 0.0;  // in degrees
    static constexpr double TURN_COMP_DISTANCE_MIN = 6.0;
    static constexpr double TURN_COMP_DISTANCE_MAX = 24.0;
    static constexpr double THROTTLE_FEED_FORWARD = 0.05;
    static constexpr double THROTTLE_MIN = -0.7;  // prac-sac
    static constexpr double THROTTLE_MAX = 0.7;   // prac-sac
    static constexpr double SKEW_MIN = -0.2;
    static constexpr double SKEW_MAX = 0.2;
    static constexpr double TURN_MIN = -0.4;
    static constexpr double TURN_MAX = 0.4;
    static constexpr double SKEW_PID_KP = 0.025;
    static constexpr double SKEW_PID_KI = 0.0;
    static constexpr double SKEW_PID_KD = 0.0;
    static constexpr double TURN_PID_KP = 0.012;  // prac-sac
    static constexpr double TURN_PID_KI = 0.0;
    static constexpr double TURN_PID_KD = 0.002;
    static constexpr double THROTTLE_PID_KP = 0.022;  // prac-sac
    static constexpr double THROTTLE_PID_KI = 0.0;
    static constexpr double THROTTLE_PID_KD = 0.003;

    static constexpr double FRONT_CARGO = 180.0;
    static constexpr double LEFT_CARGO_BAY = -90.0;
    static constexpr double RIGHT_CARGO_BAY = 90.0;
    static constexpr double LEFT_FRONT_ROCKET = -150.0;
    static constexpr double RIGHT_FRONT_ROCKET = 150.0;
    static constexpr double LEFT_BACK_ROCKET = -30.0;
    static constexpr double RIGHT_BACK_ROCKET = 30.0;
    static constexpr double HUMAN_LOADING_STATION = 0.0;

private:
    bool m_onTarget;
    double m_leftSetpoint;
    double m_rightSetpoint;
    HatchIntake *m_hatchIntake;
    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;

    double m_throttle;
    double m_turn;
    double m_gyroAngle;

    Limelight *m_limelight;

    double m_throttlePidOut;
    double m_turnPidOut;
    double m_skewPidOut;

    PID *m_turnPid;
    PID *m_throttlePid;
    PID *m_skewPid;
};
}
