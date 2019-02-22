#include "src/subsystems/HatchIntake.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/util/WrapDash.h"
#include "lib/sensors/Limelight.h"
#include "src/TeleopMode.h"

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

void HatchIntake::LaunchHatch() {
    GoToIntakeState(HatchIntakeState::launch);
}

void HatchIntake::GrabHatch() {
    GoToIntakeState(HatchIntakeState::intaking);
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
    return (m_hatchRollers->GetOutputCurrent() > 25.0);
}

void HatchIntake::GoToPneumaticState(HatchSolenoidState newState) {
    m_hatchSolenoidState = newState;
}
void HatchIntake::GoToIntakeState(HatchIntakeState newState) {
    m_hatchIntakeState = newState;
}

void HatchIntake::TaskPeriodic(RobotMode mode) {
    switch (m_hatchIntakeState) {
        case HatchIntakeState::idle:
            m_hatchRollers->Set(ControlMode::PercentOutput, 0.0);
            break;
        case HatchIntakeState::intaking:
            m_hatchRollers->Set(ControlMode::PercentOutput, -1.0);
            if (IsHatchInIntake()) {
                GoToIntakeState(HatchIntakeState::hold);
                m_limelightHatch->SetLightBlink();
                m_hatchTimer = GetMsecTime();
            }
            break;
        case HatchIntakeState::hold:
            m_hatchRollers->Set(ControlMode::PercentOutput, -0.1);
            if (m_hatchTimer - GetMsecTime() > 100) {
                m_limelightHatch->SetLightOff();
            }
            break;
        case HatchIntakeState::exhaust:
            m_hatchRollers->Set(ControlMode::PercentOutput, 1.0);
            break;
        case HatchIntakeState::launch:
            m_hatchRollers->Set(ControlMode::PercentOutput, 1.0);
            GoToPneumaticState(HatchSolenoidState::launch);
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
