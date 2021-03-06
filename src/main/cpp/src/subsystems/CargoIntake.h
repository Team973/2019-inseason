#pragma once

#include "lib/helpers/GreyCTRE.h"
#include "lib/managers/CoopTask.h"
#include "lib/managers/TaskMgr.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/filters/MovingAverageFilter.h"
#include "lib/sensors/Limelight.h"

#include "src/info/RobotInfo.h"

namespace frc973 {

/**
 * Cargo intake
 */
class CargoIntake : public CoopTask {
public:
    /**
     * Construct a CargoIntake subsystem.
     * @param scheduler The TaskMgr.
     * @param logger The LogSpreadsheet.
     * @param cargoIntakeMotor The CargoIntake GreyTalonSRX.
     * @param cargoPlatformLock The CargoIntake platform lock Solenoid.
     * @param cargoWrist The CargoIntake wrist Solenoid.
     * @param limelightHatch The hatch Limelight.
     * @param pdp The PowerDistributionPanel.
     */
    CargoIntake(TaskMgr *scheduler, LogSpreadsheet *logger,
                GreyTalonSRX *cargoIntakeMotor, Solenoid *cargoPlatformLock,
                Solenoid *cargoWrist, Limelight *limelightHatch,
                PowerDistributionPanel *pdp);
    virtual ~CargoIntake();

    /**
     * CargoIntake states.
     */
    enum class CargoIntakeState
    {
        running,       /**< Intaking state. */
        manualRunning, /**< Manual state. */
        holding,       /**< Holding state. */
        notRunning,    /**< Stopped state. */
        reverse        /**< Outtaking state. */
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

    /**
     * Run the intake with variable power.
     * @param power The power to set.
     */
    void RunIntake(double power);

    /**
     * Sets CargoIntakeState to running.
     */
    void RunIntake();

    /**
     * Sets CargoIntakeState to holding.
     */
    void HoldCargo();

    /**
     * Sets CargoIntakeState to notRunning.
     */
    void StopIntake();

    /**
     * Sets CargoIntakeState to reverse.
     */
    void Exhaust();

    /**
     * Sets CargoWristState to extended.
     */
    void ExtendWrist();

    /**
     * Sets CargoWristState to retracted.
     */
    void RetractWrist();

    /**
     * Sets CargoPlatformLockState to deployed.
     */
    void DeployPlatformWheel();

    /**
     * Sets CargoPlatformLockState to retracted.
     */
    void RetractPlatformWheel();

    /**
     * Get the intake current.
     * @return The intake motor's current pull in ohms.
     */
    double GetIntakeCurrent();

    /**
     * Get the current CargoIntakeState.
     * @return The current CargoIntakeState.
     */
    CargoIntakeState GetIntakeState();

    /**
     * Get the current CargoWristState.
     * @return The current CargoWristState.
     */
    CargoWristState GetWristState();

    /**
     * Get the current CargoPlatformWheelState.
     * @return The current CargoPlatformWheelState.
     */
    CargoPlatformLockState GetPlatformLockState();

    /**
     * Sets the CargoIntake motor to coast mode.
     */
    void EnableCoastMode();

    /**
     * Sets the CargoIntake motor to brake mode.
     */
    void EnableBrakeMode();

    /**
     * The looping task periodic.
     * @param mode The current RobotMode.
     */
    void TaskPeriodic(RobotMode mode) override;

private:
    /**
     * Sets the desired CargoIntake state
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

    LogCell *m_matchIdentifier;
    LogCell *m_batteryVoltage;
    LogCell *m_matchTime;
    LogCell *m_dateTime;

    PowerDistributionPanel *m_pdp;

    MovingAverageFilter *m_intakeCurentFilter;
};
}
