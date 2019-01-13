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
     */
    HatchIntake(TaskMgr *scheduler, LogSpreadsheet *logger);
    virtual ~HatchIntake();

    enum ClawState
    {
        open = true,
        close = false
    };

    enum PuncherState
    {
        punch = true,
        retract = false
    };

    enum class HatchIntakeState
    {
        release,
        grab,
        close,
        pushOpen,
        pushClose,
        preLaunch,
        launch,
        launchReset,
        manual
    };

    /**
     * When called, opens claw.
     */
    void ClawOpen();

    /**
     * When called, closes claw.
     */
    void ClawClose();

    /*
     * When called, activates kicker.
     */
    void PuncherOn();

    /**
     * When called, deactivates clawKicker.
     */
    void PuncherOff();

    /**
     * When called, opens the claw arms.
     */
    void HatchOpen();

    /**
     * When called, closes claw arms, and
     * has all claw solenoids set to idle
     */
    void HatchGrab();

    /**
     * When called, opens claw arms and
     * activates kicker.
     */
    void HatchPush();

    /**
     * When called, launches cube.
     */
    void HatchLaunch();

    void ManualClawOpen();
    void ManualClawClose();

    /**
     * When called, activates puncher.
     */
    void ManualPuncherOn();

    /**
     * When called, deactivates puncher.
     */
    void ManualPuncherOff();

    /**
     * Periodically update information about the drive.
     * @param mode The current robot mode.
     */
    void TaskPeriodic(RobotMode mode);

private:
    void GoToIntakeState(HatchIntakeState state);

    TaskMgr *m_scheduler;
    DigitalInput *m_rightHatchSensor;
    DigitalInput *m_leftHatchSensor;
    LogSpreadsheet *m_logger;
    Solenoid *m_hatchClaw;
    Solenoid *m_hatchPuncher;
    HatchIntakeState m_hatchIntakeState;

    uint32_t m_stateStartTimeMs;
};
}
