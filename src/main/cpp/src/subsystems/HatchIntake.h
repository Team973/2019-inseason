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
        clawOpen = true,
        clawClosed = false
    };

    enum PuncherState
    {
        active = true,
        puncherIdle = false
    };

    enum HatchIntakeState
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

    /*
     * When called, opens the claw arms
     */
    void HatchOpen();

    /*
     * When called, closes claw arms, and
     * has all claw solenoids set to idle
     */
    void HatchGrab();

    /*
     * When called, opens claw arms and
     * activates kicker
     */
    void HatchPush();

    /*
     * When called, activates kicker
     */
    void HatchPuncherOn();

    /*
     * When called, deactivates clawKicker
     */
    void HatchPuncherOff();

    /*
     * When called, launches cube
     */
    void HatchLaunch();

    /*
     * When called, closes claw
     */
    void ClawClose();

    /*
     * Whe called, opens claw
     */
    void ClawOpen();

    /*
     * When called, activates kicker
     */
    void ManualPuncherActivate();

    /*
     * When called, deactivates Kicker
     */

    void ManualPuncherIdle();

    /**
     * Periodically update information about the drive.
     * @param mode The current robot mode.
     */

    void TaskPeriodic(RobotMode mode);

private:
    void GoToState(HatchIntakeState newState);

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
