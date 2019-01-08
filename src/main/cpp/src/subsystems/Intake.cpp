#include "src/subsystems/Intake.h"

namespace frc973 {
Intake::Intake(TaskMgr *scheduler, LogSpreadsheet *logger)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_intakeMotor(new TalonSRX(INTAKE_CAN_ID))
        , m_intakeState(IntakeState::notRunning) {
    this->m_scheduler->RegisterTask("Intake", this, TASK_PERIODIC);
    m_intakeMotor->Set(ControlMode::PercentOutput, 0.0);
    m_intakeMotor->SetNeutralMode(NeutralMode::Coast);
    m_intakeMotor->EnableCurrentLimit(false);

    m_current = new LogCell("Intake Current", 32, true);
    m_logger->RegisterCell(m_current);
}

Intake::~Intake() {
    m_scheduler->UnregisterTask(this);
}

void Intake::IntakeStart() {
    m_intakeState = IntakeState::running;
}

void Intake::IntakeStartReverse() {
    m_intakeState = IntakeState::reverse;
}

void Intake::IntakeStop() {
    m_intakeState = IntakeState::notRunning;
}

void Intake::TaskPeriodic(RobotMode mode) {
    m_current->LogDouble(m_intakeMotor->GetOutputCurrent());
    switch (m_intakeState) {
        case IntakeState::running:
            m_intakeMotor->Set(ControlMode::PercentOutput, 1.0);
            break;
        case IntakeState::notRunning:
            m_intakeMotor->Set(ControlMode::PercentOutput, 0.0);
            break;
        case IntakeState::reverse:
            m_intakeMotor->Set(ControlMode::PercentOutput, -1.0);
            break;
    }
}
}
