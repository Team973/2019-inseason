#include "src/subsystems/Subsystem.h"

namespace frc973 {
Subsystem::Subsystem(TaskMgr *scheduler, LogSpreadsheet *logger,
                     GreyTalonSRX *subsystemMotor)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_subsystemMotor(subsystemMotor) {
    this->m_scheduler->RegisterTask("Subsystem", this, TASK_PERIODIC);
    m_subsystemMotor->Set(ControlMode::PercentOutput, 0.0);
    m_subsystemMotor->SetNeutralMode(NeutralMode::Coast);
    m_subsystemMotor->EnableCurrentLimit(false);

    m_current = new LogCell("Subsystem Current", 32, true);
    m_logger->RegisterCell(m_current);
}

Subsystem::~Subsystem() {
    m_scheduler->UnregisterTask(this);
}

void Subsystem::SubsystemStart() {
}

void Subsystem::SubsystemStop() {
}

void Subsystem::TaskPeriodic(RobotMode mode) {
    m_current->LogDouble(m_subsystemMotor->GetOutputCurrent());
}
}
