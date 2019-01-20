/*
 * Elevator.h
 *
 *  Created on: January 7, 2019
 *      Author: Kyle
 */

#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/helpers/GreyTalon.h"
#include "lib/managers/CoopTask.h"
#include "lib/logging/LogSpreadsheet.h"
#include "src/info/RobotInfo.h"
#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/sensors/Limelight.h"
#include "src/controllers/LimelightVerticalController.h"
#include "lib/util/Util.h"

using namespace frc;

namespace frc973 {
class TaskMgr;
class LogSpreadsheet;

/**
 * Elevator Subsystem.
 */
class Elevator : public CoopTask {
public:
    /**
     * Defines the elevator control states for Talon.
     */
    enum ElevatorState
    {
        manualVoltage, /**< Control the motors with manual voltage. */
        motionMagic, /**< Control the motors using position w/ Motion Magic. */
        limelight
    };

    static constexpr double GROUND = 0.0;       /**< Ground preset. */
    static constexpr double LOW_GOAL = 24.0;    /**< Low preset. */
    static constexpr double MIDDLE_GOAL = 48.0; /**< Middle preset. */
    static constexpr double HIGH_GOAL = 96.0;   /**< High preset. */
    static constexpr double PLATFORM = 24.0;    /**< Platform preset. */

    static constexpr double ELEVATOR_SOFT_HEIGHT_LIMIT =
        80.5; /**< Soft elevator height. */
    static constexpr double ELEVATOR_INCHES_PER_CLICK =
        8.0 / 4096.0; /**< Encoder in/click */
    static constexpr double ELEVATOR_FEED_FORWARD =
        0.1; /**< The elevator's feed forward. */

    /**
     * Contruct an elevator.
     * @param scheduler TaskMgr object.
     * @param logger LogSpreadsheet object.
     * @param elevatorMotor The elevator Talon.
     */
    Elevator(TaskMgr *scheduler, LogSpreadsheet *logger,
             GreyTalonSRX *elevatorMotorA, VictorSPX *elevatorMotorB,
             Limelight *limelight);
    virtual ~Elevator();

    /**
     * Set the elevator position using Motion Magic.
     * @param position The position goal.
     */
    void SetPosition(double position);

    /**
     * Set the elevator power.
     * @param power The power being sent to the motor from -1.0 to 1.0
     */
    void SetPower(double power);

    /**
     * Enable Limelight control.
     */
    void EnableLimelightControl();

    /**
     * Get the current position.
     * @return The current position in sensor units.
     */
    float GetPosition() const;

    /**
     * Zero the current position.
     */
    void ZeroPosition();

    /**
     * Enable brake mode on the elevator Talon.
     */
    void EnableBrakeMode();

    /**
     * Enable coast mode on the elevator Talon.
     */
    void EnableCoastMode();

    /**
     * Update function synonymous to TeleopContinuous that gets called
     * continuously.
     * @param mode The current robot mode.
     */
    void TaskPeriodic(RobotMode mode);

private:
    TaskMgr *m_scheduler;

    GreyTalonSRX *m_elevatorMotorA;
    VictorSPX *m_elevatorMotorB;

    double m_position;
    uint32_t m_zeroingTime;
    ElevatorState m_elevatorState;
    LimelightVerticalController *m_limelightVerticalController;
    LogCell *m_positionCell;
};
}
