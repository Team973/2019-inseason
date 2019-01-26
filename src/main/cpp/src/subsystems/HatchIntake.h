#pragma once

#include "frc/WPILib.h"
#include "lib/managers/CoopTask.h"
#include "lib/logging/LogSpreadsheet.h"
#include "src/info/RobotInfo.h"
#include "lib/helpers/GreyTalon.h"

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
                GreyTalonSRX *hatchRollers, Solenoid *hatchPuncher);
    virtual ~HatchIntake();

    /**
     * Hatch intake states.
     */
    enum class HatchIntakeState
    {
        idle,     /**< Idle hatch intake. */
        intaking, /**< Prime hatch intake for intaking. */
        hold,     /**< Hold the hatch. */
        exhaust,  /**< Place the hatch. */
        launch,   /**< Launch the hatch. */
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
    void SetIntaking();

    /**
     * Set hatch intake to holding.
     */
    void HoldHatch();

    /**
     * Set hatch intake to exhausting.
     */
    void Exhaust();

    /**
     * Open the claw arms.
     */
    void OpenClaw();

    /**
     * Grab the hatch.
     */
    void GrabHatch();

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

    HatchIntakeState m_hatchIntakeState;
    HatchSolenoidState m_hatchSolenoidState;

    void GoToPneumaticState(HatchSolenoidState newState);
    void GoToIntakeState(HatchIntakeState newState);

    uint32_t m_hatchSolenoidStateTimer;
    uint32_t m_hatchIntakeStateTimer;
};
}
