#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/helpers/GreyTalon.h"
#include "src/info/RobotInfo.h"
#include "lib/managers/CoopTask.h"
#include "lib/managers/TaskMgr.h"
#include "lib/logging/LogSpreadsheet.h"

using namespace frc;
using namespace ctre;

namespace frc973 {
class TaskMgr;
class LogSpreadsheet;
class LogCell;

class CargoIntake : public CoopTask {
public:
    /**
     * Construct a Cargo Intake Subsystem.
     * @param scheduler The task manager.
     * @param logger The logger.
     * @param cargoIntakeMotor The cargo intake motor.
     * @param cargoWristLock The cargo wrist lock solenoid.
     * @param cargoWrist The cargo wrist solenoid.
     * @param cargoWheelPiston The cargo platform wheel solenoid.
     */
    CargoIntake(TaskMgr *scheduler, LogSpreadsheet *logger,
                GreyTalonSRX *cargoIntakeMotor, Solenoid *cargoWristLock,
                Solenoid *cargoWrist, Solenoid *cargoWheelPiston);
    virtual ~CargoIntake();

    /**
     * Cargo Intake states.
     */
    enum class CargoIntakeState
    {
        running,    /**< Intaking state. */
        holding,    /**< Holding state. */
        notRunning, /**< Stopped state. */
        reverse     /**< Outtaking state. */
    };

    /**
     * Cargo Wrist Lock states.
     */
    enum class CargoWristLockState
    {
        unlocked, /**< Unlocked wrist state. */
        locked    /**< Locked wrist state. */
    };

    /**
     * Cargo Wrist states.
     */
    enum class CargoWristState
    {
        extended, /**< Extended wrist state. */
        retracted /**< Retracted wrist state. */
    };

    /**
     * Cargo Platform Wheel states.
     */
    enum class CargoWheelPistonState
    {
        retracted, /**< Retracted wheel state. */
        deployed   /**< Deployed wheel state. */
    };

    /**
     * Cargo Endgame state.
     */
    enum class CargoEndgameState
    {
        notEndgame, /**< Default, not endgame state. */
        stowed,     /**< Stowed state. */
        deployed    /**< Deployed state. */
    };

    void RunIntake();  /**< Set the CargoIntakeState to running. */
    void HoldCargo();  /**< Set the CargoIntakeState to holding. */
    void StopIntake(); /**< Set the CargoIntakeState to notRunning. */
    void Exhaust();    /**< Set the CargoIntakeState to reverse. */

    void UnlockWrist(); /**< Set the CargoWristLockState to unlocked. */
    void LockWrist();   /**< Set the CargoWristLockState to locked. */

    void ExtendWrist();  /**< Set the CargoWristState to extended. */
    void RetractWrist(); /**< Set the CargoWristState to retracted. */

    void DeployWheelPiston();  /**< Set the CargoWheelPistonState
                                    to deployed */
    void RetractWheelPiston(); /**< Set the CargoWheelPistonState to
                                    retracted */

    /**
     * Get the intake current.
     * @returns The intake motor's current pull in ohms.
     */
    double GetIntakeCurrent();

    /**
     * Get the current CargoIntakeState.
     * @returns The current CargoIntakeState.
     */
    CargoIntakeState GetIntakeState();

    /**
     * Get the current CargoWristLockState.
     * @returns The current CargoWristLockState.
     */
    CargoWristLockState GetWristLockState();

    /**
     * Get the current CargoWristState.
     * @returns The current CargoWristState.
     */
    CargoWristState GetWristState();

    /**
     * Get the current CargoWheelPistonState.
     * @returns The current CargoWheelPistonState.
     */
    CargoWheelPistonState GetWheelPistonState();

    /**
     * Get the current CargoEndgameState.
     * @returns The current CargoEndgameState.
     */
    CargoEndgameState GetEndgameState();

    /**
     * The looping task periodic.
     * @param mode The current robot mode.
     */
    void TaskPeriodic(RobotMode mode) override;

private:
    TaskMgr *m_scheduler;
    LogSpreadsheet *m_logger;

    GreyTalonSRX *m_cargoIntakeMotor;
    Solenoid *m_cargoWrist;
    Solenoid *m_cargoWristLock;
    Solenoid *m_cargoWheelPiston;

    uint32_t m_cargoEndgameTimer;

    CargoIntakeState m_cargoIntakeState;
    CargoWristState m_cargoWristState;
    CargoWristLockState m_cargoWristLockState;
    CargoWheelPistonState m_cargoWheelPistonState;
    CargoEndgameState m_cargoEndgameState;

    /**
     * Sets the desired cargo intake state
     * @param newState The desired Cargo Intake state.
     */
    void GoToIntakeState(CargoIntakeState newState);

    /**
     * Go to the wrist lock state
     * @param newState The Wrist Lock state.
     */
    void GoToWristLockState(CargoWristLockState newState);

    /**
     * Go to the desired wrist state
     * @param newState The desired Wrist state.
     */
    void GoToWristState(CargoWristState newState);

    /**
     * Go to the platform wheel state
     * @param newState The Platform Wheel state.
     */
    void GoToWheelPistonState(CargoWheelPistonState newState);

    /**
     * Go to the desired end game state
     * @param newState The desired End Game state.
     */
    void GoToEndgameState(CargoEndgameState newState);

    LogCell *m_current;
};
}
