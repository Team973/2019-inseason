#pragma once

#include "lib/helpers/GreyTalon.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/managers/CoopTask.h"
#include "lib/managers/TaskMgr.h"
#include "lib/util/WrapDash.h"

#include "src/info/RobotInfo.h"

namespace frc973 {

/**
 * Stinger
 */
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

    /**
     * Defines the Switch Blade States
     */
    enum class SwitchBladeState
    {
        engaged,  /**< Switch Blade Extended*/
        retracted /**< Switch Blade Stowed*/
    };

    /**
     * Returns the Switch Blade's State.
     * @return The Switch Blade's State.
     */
    SwitchBladeState GetSwitchBladeState();

    /**
     * Deploys the stingers switch blade
     */
    void DeploySwitchBlade();

    /**
     * Retracts the stinger's switch blade
     */
    void RetractSwitchBlade();

    /**
     * Engages the stinger's gate latch
     */
    void EngageGateLatch();

    /**
     * Retracts the gate latch.
     */
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
