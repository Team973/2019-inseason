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

class CargoIntake : public CoopTask {
public:
    CargoIntake(TaskMgr *scheduler, LogSpreadsheet *logger);
    virtual ~CargoIntake();

    void Start();
    void Stop();
    void Exhaust();

    void ExtendWrist();
    void RetractWrist();

    void TaskPeriodic(RobotMode mode) override;

    enum class IntakeState
    {
        running,
        notRunning,
        reverse,
        hold
    };

private:
    TaskMgr *m_scheduler;
    LogSpreadsheet *m_logger;
    TalonSRX *m_intakeMotor;
    Solenoid *m_wrist;
    IntakeState m_intakeState;

    LogCell *m_current;
};
}
