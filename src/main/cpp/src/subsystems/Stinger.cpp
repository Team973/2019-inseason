#include "src/subsystems/Stinger.h"

namespace frc973 {
Stinger::Stinger(TaskMgr *scheduler, LogSpreadsheet *logger,
                 GreyTalonSRX *stingerElevatorMotor)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_stingerElevatorMotor(stingerElevatorMotor) {
    this->m_scheduler->RegisterTask("Stinger", this, TASK_PERIODIC);
    m_stingerElevatorMotor->Set(ControlMode::PercentOutput, 0.0);
    m_stingerElevatorMotor->SetNeutralMode(NeutralMode::Coast);
    m_stingerElevatorMotor->EnableCurrentLimit(false);

    m_current = new LogCell("Stinger Current", 32, true);
    m_logger->RegisterCell(m_current);
}

Stinger::~Stinger() {
    m_scheduler->UnregisterTask(this);
}

void Stinger::StingerStart() {
}

void Stinger::StingerStop() {
}

void Stinger::TaskPeriodic(RobotMode mode) {
    m_current->LogDouble(m_stingerElevatorMotor->GetOutputCurrent());
}
}
