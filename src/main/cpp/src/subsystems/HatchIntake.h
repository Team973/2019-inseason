#pragma once

#include "lib/helpers/GreyCTRE.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/managers/CoopTask.h"
#include "lib/sensors/Limelight.h"

namespace frc973 {

/**
 * Hatch intake
 */
class HatchIntake : public CoopTask {
public:
    /**
     * Hatch intake constructor.
     * @param scheduler TaskMgr object.
     * @param logger LogSpreadsheet object.
     * @param hatchRollers The HatchIntake's roller GreyTalonSRX.
     * @param hatchPuncher The HatchIntake's puncher Solenoid.
     * @param limelightHatch The HatchIntake's Limelight.
     */
    HatchIntake(TaskMgr *scheduler, LogSpreadsheet *logger,
                GreyTalonSRX *hatchRollers, Solenoid *hatchPuncher,
                Limelight *limelightHatch);
    virtual ~HatchIntake();

    /**
     * Hatch intake states.
     */
    enum class HatchIntakeState
    {
        idle,     /**< Idle hatch intaking. */
        intaking, /**< Prime HatchIntake for intaking. */
        hold,     /**< Hold the hatch. */
        exhaust,  /**< Place the hatch. */
        manual    /**< Manual HatchIntake control. */
    };

    /**
     * Hatch intake pneumatic states.
     */
    enum class HatchSolenoidState
    {
        launch,        /**< Launch the hatch. */
        manualPunch,   /**< Manually punch the hatch. */
        manualRetract, /**< Manually retract the puncher. */
        manual         /**< Manual pneumatic control. */
    };

    /**
     * Hatch puncher Solenoid definitions.
     */
    enum HatchPuncherSolenoidState
    {
        punch = true,   /**< Punches hatch. */
        retract = false /**< Retract puncher. */
    };

    /**
     * Set HatchIntake to HatchIntakeState.idle.
     */
    void SetIdle();

    /**
     * Set HatchIntake to HatchIntakeState.intaking.
     */
    void RunIntake();

    /**
     * Set HatchIntake to HatchIntakeState.hold.
     */
    void HoldHatch();

    /**
     * Set HatchIntake to HatchIntakeState.exhaust.
     */
    void Exhaust();

    /**
     * Set HatchIntake to HatchSolenoidState.launch.
     */
    void LaunchHatch();

    /**
     * Checks if a hatch is in the intake.
     * @return Whether the hatch is in the intake.
     */
    bool IsHatchInIntake();

    /**
     * Set HatchIntake to HatchSolenoidState.manualPunch, regardless of grab
     * state.
     */
    void ManualPuncherActivate();

    /**
     * Set HatchIntake to HatchSolenoidState.manualRetract, regardless of
     * grab state.
     */
    void ManualPuncherRetract();

    /**
     * Gets the current HatchSolenoidState.
     * @return The current HatchSolenoidState.
     */
    HatchSolenoidState GetHatchPuncherState();

    /**
     * Periodically update information about the HatchIntake.
     * @param mode The current RobotMode.
     */
    void TaskPeriodic(RobotMode mode);

private:
    TaskMgr *m_scheduler;
    LogSpreadsheet *m_logger;

    GreyTalonSRX *m_hatchRollers;
    Solenoid *m_hatchPuncher;

    Limelight *m_limelightHatch;

    HatchIntakeState m_hatchIntakeState;
    HatchSolenoidState m_hatchSolenoidState;

    LogCell *m_currentCell;
    LogCell *m_voltageCell;

    void GoToPneumaticState(HatchSolenoidState newState);
    void GoToIntakeState(HatchIntakeState newState);

    double m_hatchTimer;
};
}
