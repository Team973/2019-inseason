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

class Intake : public CoopTask {
public:
    Intake(TaskMgr *scheduler, LogSpreadsheet *logger);
    virtual ~Intake();

    void IntakeStart();
    void IntakeStop();
    void IntakeStartReverse();

    void TaskPeriodic(RobotMode mode) override;

    enum class IntakeState
    {
        running,
        notRunning,
        reverse
    };

private:
    TaskMgr *m_scheduler;
    LogSpreadsheet *m_logger;
    TalonSRX *m_intakeMotor;
    IntakeState m_intakeState;

    LogCell *m_current;
};
}
