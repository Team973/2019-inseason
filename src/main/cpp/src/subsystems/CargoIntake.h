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

class CargoIntake : public CoopTask {
public:
    CargoIntake(TaskMgr *scheduler, LogSpreadsheet *logger);
    virtual ~CargoIntake();

    void RunIntake(double power);
    void Stop();
    void Exhaust(double power);

    void ExtendWrist();
    void RetractWrist();

    double GetCurrent();

    void TaskPeriodic(RobotMode mode) override;

private:
    enum WristState
    {
        extend = true,
        retract = false
    };

    TaskMgr *m_scheduler;
    LogSpreadsheet *m_logger;
    GreyTalonSRX *m_intakeMotor;
    Solenoid *m_wrist;
    IntakeState m_intakeState;

    LogCell *m_current;
};
}
