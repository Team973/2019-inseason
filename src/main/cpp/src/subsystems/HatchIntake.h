#pragma once

#include "frc/WPILib.h"
#include "lib/managers/CoopTask.h"
#include "lib/logging/LogSpreadsheet.h"
#include "src/info/RobotInfo.h"

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
     * @param hatchClaw The hatch intake's claw.
     * @param hatchPuncher The hatch intake's puncher.
     * @param leftHatchSensor The hatch intake's left hatch sensor.
     * @param rightHatchSensor The hatch intake's right hatch sensor.
     */
    HatchIntake(TaskMgr *scheduler, LogSpreadsheet *logger, Solenoid *hatchClaw,
                Solenoid *hatchPuncher, DigitalInput *leftHatchSensor,
                DigitalInput *rightHatchSensor);
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
        manual    /**< Manual hatch intake control. */
    };

    /**
     * Hatch intake pneumatic states.
     */
    enum class HatchPneumaticState
    {
        release,       /**< Release the hatch. */
        grab,          /**< Grab the hatch. */
        launch,        /**< Launch the hatch. */
        manualPunch,   /**< Manually punch the hatch. */
        manualRetract, /**< Manually retract the puncher. */
        manual         /**< Manual pneumatic control. */
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
    void Exhast();

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

    Solenoid *m_hatchClaw;
    Solenoid *m_hatchPuncher;
    DigitalInput *m_leftHatchSensor;
    DigitalInput *m_rightHatchSensor;

    HatchIntakeState m_hatchIntakeState;
    HatchPneumaticState m_hatchPneumaticState;

    void GoToPneumaticState(HatchPneumaticState newState);
    void GoToIntakeState(HatchIntakeState newState);

    uint32_t m_hatchPneumaticStateTimer;
};
}
