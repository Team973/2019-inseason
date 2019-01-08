#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
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

class Subsystem : public CoopTask {
public:
    Subsystem(TaskMgr *scheduler, LogSpreadsheet *logger);
    virtual ~Subsystem();

    void SubsystemStart();
    void SubsystemStop();

    void TaskPeriodic(RobotMode mode) override;

private:
    TaskMgr *m_scheduler;
    LogSpreadsheet *m_logger;
    TalonSRX *m_subsystemMotor;

    LogCell *m_current;
};
}
