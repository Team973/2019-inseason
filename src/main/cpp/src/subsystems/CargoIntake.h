#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/helpers/GreyTalon.h"
#include "src/info/RobotInfo.h"
#include "lib/managers/CoopTask.h"
#include "lib/managers/TaskMgr.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/filters/MovingAverageFilter.h"
#include "lib/sensors/Limelight.h"

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
                GreyTalonSRX *cargoIntakeMotor, Solenoid *cargoPlatformLock,
                Solenoid *cargoWrist, Limelight *limelightHatch);
    virtual ~CargoIntake();

    /**
     * Cargo Intake states.
     */
    enum class CargoIntakeState
    {
        running, /**< Intaking state. */
        manualRunning,
        holding,    /**< Holding state. */
        notRunning, /**< Stopped state. */
        reverse     /**< Outtaking state. */
    };
    /**
     * Cargo Wrist states.
     */
    enum class CargoWristState
    {
        extended = true,  /**< Extended wrist state. */
        retracted = false /**< Retracted wrist state. */
    };

    /**
     * Cargo Platform Wheel states.
     */
    enum class CargoPlatformLockState
    {
        retracted = true, /**< Retracted wheel state. */
        deployed = false  /**< Deployed wheel state. */
    };

    void RunIntake(double power); /**< Run the intake with variable power.*/
    void RunIntake();             /**< Set the CargoIntakeState to running. */
    void HoldCargo();             /**< Set the CargoIntakeState to holding. */
    void StopIntake(); /**< Set the CargoIntakeState to notRunning. */
    void Exhaust();    /**< Set the CargoIntakeState to reverse. */

    void ExtendWrist();  /**< Set the CargoWristState to extended. */
    void RetractWrist(); /**< Set the CargoWristState to retracted. */

    void DeployPlatformWheel();  /**< Set the CargoPlatformLockState
                                    to deployed */
    void RetractPlatformWheel(); /**< Set the CargoPlatformLockState to
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
     * Get the current CargoWristState.
     * @returns The current CargoWristState.
     */
    CargoWristState GetWristState();

    /**
     * Get the current CargoPlatformWheelState.
     * @returns The current CargoPlatformWheelState.
     */
    CargoPlatformLockState GetPlatformLockState();

    void EnableCoastMode();
    void EnableBrakeMode();

    /**
     * The looping task periodic.
     * @param mode The current robot mode.
     */
    void TaskPeriodic(RobotMode mode) override;

private:
    /**
     * Sets the desired cargo intake state
     * @param newState The desired Cargo Intake state.
     */
    void GoToIntakeState(CargoIntakeState newState);

    /**
     * Go to the desired wrist state
     * @param newState The desired Wrist state.
     */
    void GoToWristState(CargoWristState newState);

    /**
     * Go to the platform lock state
     * @param newState The Platform Wheel state.
     */
    void GoToPlatformLockState(CargoPlatformLockState newState);

    TaskMgr *m_scheduler;
    LogSpreadsheet *m_logger;

    GreyTalonSRX *m_cargoIntakeMotor;
    Solenoid *m_cargoWrist;
    Solenoid *m_cargoPlatformLock;

    double m_power;

    CargoIntakeState m_cargoIntakeState;
    CargoWristState m_cargoWristState;
    CargoPlatformLockState m_cargoPlatformLockState;

    Limelight *m_limelightHatch;
    double m_cargoTimer;

    LogCell *m_currentCell;
    LogCell *m_voltageCell;

    MovingAverageFilter *m_intakeCurentFilter;
};
}
