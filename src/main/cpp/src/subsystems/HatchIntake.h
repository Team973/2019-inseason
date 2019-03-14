#pragma once

#include "frc/WPILib.h"
#include "lib/managers/CoopTask.h"
#include "lib/logging/LogSpreadsheet.h"
#include "src/info/RobotInfo.h"
#include "lib/helpers/GreyTalon.h"
#include "lib/sensors/Limelight.h"

using namespace frc;

namespace frc973 {
class TaskMgr;
class LogSpreadsheet;
class LogCell;

class HatchIntake : public CoopTask {
public:
    /**
     * Hatch intake constructor.
     * @param scheduler TaskMgr object.
     * @param logger LogSpreadsheet object.
     * @param hatchRoller The hatch intake's roller.
     * @param hatchPuncher The hatch intake's puncher.
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
        idle,     /**< Idle hatch intakeintaking. */
        intaking, /**< Prime hatch intake for intaking. */
        hold,     /**< Hold the hatch. */
        exhaust,  /**< Place the hatch. */
        manual    /**< Manual hatch intake control. */
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
     * Hatch puncher solenoid definitions.
     */
    enum HatchPuncherSolenoidStates
    {
        punch = true,   /**< Punches hatch. */
        retract = false /**< Retract puncher. */
    };

    /**
     * Set hatch intake to idle.
     */
    void SetIdle();

    /**
     * Set hatch intake to intaking.
     */
    void RunIntake();

    /**
     * Set hatch intake to holding.
     */
    void HoldHatch();

    /**
     * Set hatch intake to exhausting.
     */
    void Exhaust();

    /**
     * Launch the hatch.
     */
    void LaunchHatch();

    bool IsHatchInIntake();

    /**
     * Manually punch, regardless of grab state.
     */
    void ManualPuncherActivate();

    /**
     * Manually retract puncher, regardless of grab state.
     */
    void ManualPuncherRetract();

    HatchSolenoidState GetHatchPuncherState();

    /**
     * Periodically update information about the hatch intake.
     * @param mode The current robot mode.
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
