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
     * @param cargoPlatformWheel The cargo platform wheel solenoid.
     */
    CargoIntake(TaskMgr *scheduler, LogSpreadsheet *logger,
                GreyTalonSRX *cargoIntakeMotor, Solenoid *cargoWristLock,
                Solenoid *cargoWrist, Solenoid *cargoPlatformWheel);
    virtual ~CargoIntake();

    /**
     * Cargo Intake states.
     */
    enum class CargoIntakeState
    {
        running,    /**< Intaking state. */
        notRunning, /**< Stopped state. */
        reverse,    /**< Outtaking state. */
        platform    /**< Endgame Platform state. */
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
    enum class CargoPlatformWheelState
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
    void StopIntake(); /**< Set the CargoIntakeState to stop. */
    void Exhaust();    /**< Set the CargoIntakeState to reverse. */

    void UnlockWrist(); /**< Set the CargoWristLockState to unlocked. */
    void LockWrist();   /**< Set the CargoWristLockState to locked. */

    void ExtendWrist();  /**< Set the CargoWristState to extended. */
    void RetractWrist(); /**< Set the CargoWristState to retracted. */

    void DeployPlatformWheel();  /**< Set the CargoPlatformWheelState to
                                    deployed */
    void RetractPlatformWheel(); /**< Set the CargoPlatformWheelState to
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
     * Get the current CargoPlatformWheelState.
     * @returns The current CargoPlatformWheelState.
     */
    CargoPlatformWheelState GetPlatformWheelState();

    /**
     * Get the current CargoEndgameState.
     * @returns The current CargoEndgameState.
     */
    CargoEndgameState GetEndgameState();

    void GoToIntakeState(CargoIntakeState newState);
    void GoToWristLockState(CargoWristLockState newState);
    void GoToWristState(CargoWristState newState);
    void GoToPlatformWheelState(CargoPlatformWheelState newState);
    void GoToEndgameState(CargoEndgameState newState);

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
    Solenoid *m_cargoPlatformWheel;

    uint32_t m_cargoEndgameTimer;

    CargoIntakeState m_cargoIntakeState;
    CargoWristState m_cargoWristState;
    CargoWristLockState m_cargoWristLockState;
    CargoPlatformWheelState m_cargoPlatformWheelState;
    CargoEndgameState m_cargoEndgameState;

    LogCell *m_current;
};
}
