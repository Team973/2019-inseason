#include "src/subsystems/CargoIntake.h"

namespace frc973 {
CargoIntake::CargoIntake(TaskMgr *scheduler, LogSpreadsheet *logger)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_intakeMotor(new TalonSRX(CARGO_INTAKE_CAN_ID))
        , m_wrist(new Solenoid(PCM_CAN_ID, CARGO_INTAKE_WRIST_PCM_ID)) {
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

void CargoIntake::RunIntake(double power) {
    m_intakeMotor->Set(ControlMode::PercentOutput, power);
}

void CargoIntake::Exhaust(double power) {
    this->RunIntake(power);
}

void CargoIntake::Stop() {
    this->RunIntake(0.0);
}

void CargoIntake::ExtendWrist() {
    m_wrist->Set(extend);
}

void CargoIntake::RetractWrist() {
    m_wrist->Set(retract);
}

double CargoIntake::GetCurrent() {
    return m_intakeMotor->GetOutputCurrent();
}

void CargoIntake::TaskPeriodic(RobotMode mode) {
    m_current->LogDouble(m_intakeMotor->GetOutputCurrent());
}
}
