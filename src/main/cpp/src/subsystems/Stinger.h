#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/helpers/GreyTalon.h"
#include "src/info/RobotInfo.h"
#include "lib/managers/CoopTask.h"
#include "lib/managers/TaskMgr.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/util/WrapDash.h"

using namespace frc;
using namespace ctre;

namespace frc973 {
class TaskMgr;
class LogSpreadsheet;
class LogCell;

class Stinger : public CoopTask {
public:
    /**
     * Construct a stinger.
     * @param scheduler The task manager.
     * @param logger The logger.
     * @param stingerDriveMotor The stingers drive motor.
     */
    Stinger(TaskMgr *scheduler, LogSpreadsheet *logger,
            GreyTalonSRX *stingerDriveMotor);
    virtual ~Stinger();

    /**
     * Deploys the stinger's kick stands
     */
    void SetKickUpEnable();

    /**
     * Retracts the stinger's kick stands
     */
    void SetKickUpDisable();

    enum class SwitchBladeState
    {
        engaged,
        retracted
    };

    SwitchBladeState GetSwitchBladeState();

    /**
     * Deploys the stingers switch blade
     */
    void DeploySwitchBlade();

    /**
     * Retracts the stinger's switch blade
     */
    void RetractSwitchBlade();

    void EngageGateLatch();
    void RetractGateLatch();

    /**
     * The periodic loooping task for the stinger elevator.
     * @param mode The current robot mode.
     */
    void TaskPeriodic(RobotMode mode) override;

private:
    TaskMgr *m_scheduler;
    LogSpreadsheet *m_logger;
    GreyTalonSRX *m_stingerDriveMotor;

    SwitchBladeState m_switchBladeState;

    DoubleSolenoid *m_kickOffPneumatic;
    DoubleSolenoid *m_sneakyClimb;
    Solenoid *m_gateLatch;

    LogCell *m_current;
};
}
