#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/helpers/GreyTalon.h"
#include "src/info/RobotInfo.h"
#include "lib/managers/CoopTask.h"
#include "lib/managers/TaskMgr.h"
#include "lib/logging/LogSpreadsheet.h"

using namespace frc;
using namespace ctre;

namespace frc973 {
class TaskMgr;
class LogSpreadsheet;
class LogCell;

class Stinger : public CoopTask {
public:
    Stinger(TaskMgr *scheduler, LogSpreadsheet *logger,
            GreyTalonSRX *stingerElevatorMotor, DigitalInput *stingerLowerHall,
            DigitalInput *stingerUpperHall);
    virtual ~Stinger();

    enum class StingerElevatorStatus
    {
        top,
        middle,
        bottom,
        error
    };

    void StingerStart();
    void StingerStop();

    bool GetLowerHall();
    bool GetUpperHall();
    StingerElevatorStatus GetStingerElevatorState();

    void SetStingerElevatorOutput(double input);

    void SetManual();

    void TaskPeriodic(RobotMode mode) override;

private:
    TaskMgr *m_scheduler;
    LogSpreadsheet *m_logger;
    GreyTalonSRX *m_stingerElevatorMotor;
    DigitalInput *m_stingerLowerHall;
    DigitalInput *m_stingerUpperHall;

    LogCell *m_current;
};
}
