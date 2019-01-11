#include "src/subsystems/CargoIntake.h"

namespace frc973 {
CargoIntake::CargoIntake(TaskMgr *scheduler, LogSpreadsheet *logger)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_intakeMotor(new TalonSRX(CARGO_INTAKE_CAN_ID))
        , m_wrist(new Solenoid(PCM_CAN_ID, CARGO_INTAKE_WRIST_PCM_ID))
        , m_intakeState(IntakeState::notRunning) {
    this->m_scheduler->RegisterTask("CargoIntake", this, TASK_PERIODIC);
    m_intakeMotor->Set(ControlMode::PercentOutput, 0.0);
    m_intakeMotor->SetNeutralMode(NeutralMode::Coast);
    m_intakeMotor->EnableCurrentLimit(false);

    m_current = new LogCell("CargoIntake Current", 32, true);
    m_logger->RegisterCell(m_current);
}

CargoIntake::~CargoIntake() {
    m_scheduler->UnregisterTask(this);
}

void CargoIntake::Start() {
    m_intakeState = IntakeState::running;
}

void CargoIntake::Exhaust() {
    m_intakeState = IntakeState::reverse;
}

void CargoIntake::Stop() {
    m_intakeState = IntakeState::notRunning;
}

void CargoIntake::ExtendWrist() {
    m_wrist->Set(true);
}

void CargoIntake::RetractWrist() {
    m_wrist->Set(false);
}

void CargoIntake::TaskPeriodic(RobotMode mode) {
    m_current->LogDouble(m_intakeMotor->GetOutputCurrent());
    switch (m_intakeState) {
        case IntakeState::running:
            ExtendWrist();
            m_intakeMotor->Set(ControlMode::PercentOutput, 1.0);
            if (m_intakeMotor->GetOutputCurrent() > 5.0) {
                m_intakeState = IntakeState::hold;
            }
            break;
        case IntakeState::notRunning:
            m_intakeMotor->Set(ControlMode::PercentOutput, 0.0);
            break;
        case IntakeState::reverse:
            m_intakeMotor->Set(ControlMode::PercentOutput, -1.0);
            break;
        case IntakeState::hold:
            m_intakeMotor->Set(ControlMode::PercentOutput, 0.2);
            RetractWrist();
            break;
    }
}
}
