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
#include "lib/util/Util.h"
#include <math.h>

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
        idle,        /**< staying in place after pressing a button. */
        joystickControl /**< Controlling with the joystick. */
    };

    static constexpr double GROUND = 0.2; /**< Ground preset. */
    static constexpr double LOW_ROCKET_HATCH =
        0.5; /**< Low rocket hatch preset. */
    static constexpr double LOW_ROCKET_CARGO =
        4.0; /**< Low rocket cargo preset. */
    static constexpr double MID_ROCKET_HATCH = 27.2;
    static constexpr double MID_ROCKET_CARGO = 27.2;
    static constexpr double LOADING_STATION_CARGO = 20.0;
    static constexpr double CARGO_SHIP_HATCH =
        0.5; /**< Cargo ship hatch preset. */
    static constexpr double CARGO_SHIP_CARGO =
        15.0; /**< Cargo ship cargo preset. */
    static constexpr double THIRD_PLATFORM = 27.0;  /**< Platform preset. */
    static constexpr double SECOND_PLATFORM = 10.0; /**< Platform preset. */

    static constexpr double ELEVATOR_HEIGHT_SOFT_LIMIT =
        27.5; /**< Soft elevator height. */
    static constexpr double ENDGAME_HEIGHT_SOFT_LIMIT = 24.25;
    static constexpr double ELEVATOR_HALL_HEIGHT_OFFSET = 0.6;
    static constexpr double ELEVATOR_INCHES_PER_CLICK =
        4.0 / 4096.0; /**< Encoder in/click */
    static constexpr double ELEVATOR_FEED_FORWARD =
        0.06; /**< The elevator's feed forward. */

    /**
     * Contruct an elevator.
     * @param scheduler TaskMgr object.
     * @param logger LogSpreadsheet object.
     * @param elevatorMotorA The elevator Talon.
     * @param elevatorMotorB The elevator victor.
     * @param operatorJoystick The codriver controller.
     * @param elevatorHall The elevators hall.
     */
    Elevator(TaskMgr *scheduler, LogSpreadsheet *logger,
             GreyTalonSRX *elevatorMotorA, VictorSPX *elevatorMotorB,
             ObservableXboxJoystick *operatorJoystick,
             DigitalInput *elevatorHall);
    virtual ~Elevator();

    /**
     * Set the elevator position using Motion Magic.
     * @param position The position goal.
     */
    void SetPosition(double position);

    /**
     * Set to manual mode with joystick control
     */
    void SetManualInput();

    /**
     * Set the elevator power.
     * @param power The power being sent to the motor from -1.0 to 1.0
     */
    void SetPower(double power);

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
     * Gets the state of the elevator hall
     * @Return The state of the hall
     */
    bool GetElevatorHall();

    /**
     * Sets the elevators soft limit
     * @param limit The soft limit to set
     */
    void SetSoftLimit(double limit);

    /**
     * Checks for halls state and auto zeros if its false
     */
    void HallZero();

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
    ObservableXboxJoystick *m_operatorJoystick;

    double m_position;
    double m_power;
    double m_joystickControl;
    bool m_prevHall;
    uint32_t m_zeroingTime;

    ElevatorState m_elevatorState;

    LogCell *m_positionCell;
    LogCell *m_currentMasterCell;
    LogCell *m_voltageMasterCell;
    LogCell *m_currentFollowerCell;
    LogCell *m_voltageFollowerCell;
    LogCell *m_controlModeCell;
    LogCell *m_powerInputCell;

    DigitalInput *m_elevatorHall;
};
}
