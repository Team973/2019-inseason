#pragma once

#include "lib/helpers/GreyCTRE.h"
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
     * Construct a Stinger.
     * @param scheduler The TaskMgr.
     * @param logger The LogSpreadsheet.
     * @param stingerDriveMotor The stinger's GreyTalonSRX.
     */
    Stinger(TaskMgr *scheduler, LogSpreadsheet *logger,
            GreyTalonSRX *stingerDriveMotor);
    virtual ~Stinger();

    /**
     * Deploys the Stinger's kick stands.
     */
    void SetKickUpEnable();

    /**
     * Retracts the Stinger's kick stands.
     */
    void SetKickUpDisable();

    /**
     * Defines the Switch Blade States.
     */
    enum class SwitchBladeState
    {
        engaged,  /**< Switch Blade Extended. */
        retracted /**< Switch Blade Stowed. */
    };

    /**
     * Get the current SwitchBladeState.
     * @return The current SwitchBladeState.
     */
    SwitchBladeState GetSwitchBladeState();

    /**
     * Set Stinger to SwitchBladeState.engaged.
     */
    void DeploySwitchBlade();

    /**
     * Set Stinger to SwitchBladeState.retracted.
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
     * The periodic loooping task for the Stinger.
     * @param mode The current RobotMode.
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
