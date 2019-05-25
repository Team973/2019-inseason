#include "src/subsystems/HatchIntake.h"

namespace frc973 {
HatchIntake::HatchIntake(TaskMgr *scheduler, LogSpreadsheet *logger,
                         GreyTalonSRX *hatchRollers, Solenoid *hatchPuncher,
                         Limelight *limelightHatch)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_hatchRollers(hatchRollers)
        , m_hatchPuncher(hatchPuncher)
        , m_hatchSolenoidState(HatchSolenoidState::manual)
        , m_hatchIntakeState(HatchIntakeState::idle)
        , m_limelightHatch(limelightHatch)
        , m_hatchTimer(0.0) {
    this->m_scheduler->RegisterTask("HatchIntake", this, TASK_PERIODIC);
    m_hatchRollers->Set(ControlMode::PercentOutput, 0.0);
    m_hatchRollers->SetNeutralMode(NeutralMode::Coast);

    m_hatchRollers->EnableCurrentLimit(false);
    m_hatchRollers->ConfigPeakCurrentDuration(0, 10);
    m_hatchRollers->ConfigPeakCurrentLimit(0, 10);
    m_hatchRollers->ConfigContinuousCurrentLimit(100, 10);
    m_hatchRollers->EnableVoltageCompensation(false);
    m_hatchRollers->SetInverted(false);
    m_hatchRollers->ConfigNeutralDeadband(0.01);

    m_currentCell = new LogCell("Hatch Current", 32, true);
    m_voltageCell = new LogCell("Hatch Voltage", 32, true);
    logger->RegisterCell(m_currentCell);
    logger->RegisterCell(m_voltageCell);
}

HatchIntake::~HatchIntake() {
    m_scheduler->UnregisterTask(this);
}

void HatchIntake::SetIdle() {
    GoToIntakeState(HatchIntakeState::idle);
}

void HatchIntake::RunIntake() {
    GoToIntakeState(HatchIntakeState::intaking);
}

void HatchIntake::HoldHatch() {
    GoToIntakeState(HatchIntakeState::hold);
}

void HatchIntake::Exhaust() {
    GoToIntakeState(HatchIntakeState::exhaust);
}

void HatchIntake::ManualPuncherActivate() {
    GoToPneumaticState(HatchSolenoidState::manualPunch);
    GoToIntakeState(HatchIntakeState::manual);
}

void HatchIntake::ManualPuncherRetract() {
    GoToPneumaticState(HatchSolenoidState::manualRetract);
    GoToIntakeState(HatchIntakeState::manual);
}

bool HatchIntake::IsHatchInIntake() {
    return (m_hatchRollers->GetOutputCurrent() > 32.0);
}

void HatchIntake::GoToPneumaticState(HatchSolenoidState newState) {
    m_hatchSolenoidState = newState;
}
void HatchIntake::GoToIntakeState(HatchIntakeState newState) {
    m_hatchIntakeState = newState;
}

HatchIntake::HatchSolenoidState HatchIntake::GetHatchPuncherState() {
    return m_hatchSolenoidState;
}

void HatchIntake::TaskPeriodic(RobotMode mode) {
    m_currentCell->LogDouble(m_hatchRollers->GetOutputCurrent());
    m_voltageCell->LogDouble(m_hatchRollers->GetMotorOutputVoltage());
    switch (m_hatchIntakeState) {
        case HatchIntakeState::idle:
            m_hatchRollers->Set(ControlMode::PercentOutput, 0.0);
            break;
        case HatchIntakeState::intaking:
            m_hatchRollers->Set(ControlMode::PercentOutput, -1.0);
            if (IsHatchInIntake()) {
                m_limelightHatch->SetLightBlink();
            }
            break;
        case HatchIntakeState::hold:
            m_hatchRollers->Set(ControlMode::PercentOutput, -0.1);
            break;
        case HatchIntakeState::exhaust:
            m_hatchRollers->Set(ControlMode::PercentOutput, 1.0);
            break;
        case HatchIntakeState::manual:
            // Do Nothing
            break;
    }

    switch (m_hatchSolenoidState) {
        case HatchSolenoidState::launch:
            break;
        case HatchSolenoidState::manualPunch:
            m_hatchPuncher->Set(HatchPuncherSolenoidStates::punch);
            break;
        case HatchSolenoidState::manualRetract:
            m_hatchPuncher->Set(HatchPuncherSolenoidStates::retract);
            break;
        case HatchSolenoidState::manual:
            // Do nothing
            break;
    }
}
}
