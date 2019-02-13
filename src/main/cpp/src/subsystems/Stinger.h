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
     * @param stingerElevatorMotor The stinger elevator motor.
     * @param stingerLowerHall The stinger lower hall.
     * @param stingerUpperHall The stinger upper hall.
     */
    Stinger(TaskMgr *scheduler, LogSpreadsheet *logger,
            GreyTalonSRX *stingerElevatorMotor, GreyTalonSRX *stingerDriveMotor,
            DigitalInput *stingerLowerHall, DigitalInput *stingerUpperHall);
    virtual ~Stinger();

    /**
     * Defines the elevator control states for Talon.
     */
    enum class StingerState
    {
        manualVoltage, /**< Control the motors with manual voltage. */
        motionMagic, /**< Control the motors using position w/ Motion Magic. */
        idle         /**< Stays in the position that was assigned. */
    };

    static constexpr double STOW = 0.0;     /**< Stow preset. */
    static constexpr double MIDDLE = -5.0;  /**< Middle preset. */
    static constexpr double BOTTOM = -10.0; /**< Bottom preset. */

    static constexpr double STINGER_SOFT_HEIGHT_LIMIT =
        20.0; /**< Soft stinger height. */
    static constexpr double STINGER_INCHES_PER_CLICK =
        (1.0 / 4096.0) * (18.0 / 50.0) * 22.0 * 0.25; /**< Encoder in/click */
    static constexpr double STINGER_FEED_FORWARD =
        0.1; /**< The stinger's feed forward. */

    /**
     * Stinger elevator hall states.
     */
    enum class StingerElevatorHallState
    {
        top,    /**< Stinger elevator triggering top hall. */
        middle, /**< Stinger elevator triggering no halls. */
        bottom, /**< Stinger elevator triggering bottom hall. */
        error   /**< Stinger elevator triggering both halls. */
    };

    /**
     * Set the manual percent output of the stinger elevator.
     * @param power The power to output.
     */
    void SetPower(double power);

    /**
     * Set the postion of the stinger elevator.
     * @param position The position preset in inches.
     */
    void SetPositionInches(double position);

    void Stow();      /**< Set the stinger elevator to the stowed preset. */
    void SetMiddle(); /**< Set the stinger elevator to the middle preset. */
    void Deploy();    /**< Set the stinger elevator to the bottom preset. */

    /**
     * Get the status of the lower hall.
     * @return The state of the lower hall (true/false).
     */
    bool GetLowerHall();

    /**
     * Get the status of the upper hall.
     * @return The state of the upper hall (true/false).
     */
    bool GetUpperHall();

    /**
     * Get the status of the stinger elevator halls.
     * @return The state of the stinger elevator halls
     * (top/middle/bottom/error).
     */
    StingerElevatorHallState GetStingerElevatorHallState();

    /**
     * The periodic loooping task for the stinger elevator.
     * @param mode The current robot mode.
     */
    void TaskPeriodic(RobotMode mode) override;

private:
    void GoToStingerState(StingerState newState);

    TaskMgr *m_scheduler;
    LogSpreadsheet *m_logger;
    GreyTalonSRX *m_stingerElevatorMotor;
    GreyTalonSRX *m_stingerDriveMotor;
    DigitalInput *m_stingerLowerHall;
    DigitalInput *m_stingerUpperHall;

    double m_power;

    LogCell *m_current;

    StingerState m_stingerState;
};
}
