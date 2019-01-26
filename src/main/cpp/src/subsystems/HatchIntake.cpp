#include "src/subsystems/HatchIntake.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/util/WrapDash.h"

namespace frc973 {
HatchIntake::HatchIntake(TaskMgr *scheduler, LogSpreadsheet *logger,
                         GreyTalonSRX *hatchRollers, Solenoid *hatchPuncher)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_hatchRollers(hatchRollers)
        , m_hatchPuncher(hatchPuncher)
        , m_hatchSolenoidState(HatchSolenoidState::manual)
        , m_hatchIntakeState(HatchIntakeState::idle) {
    this->m_scheduler->RegisterTask("HatchIntake", this, TASK_PERIODIC);
}

HatchIntake::~HatchIntake() {
    m_scheduler->UnregisterTask(this);
}

void HatchIntake::SetIdle() {
    GoToIntakeState(HatchIntakeState::idle);
}

void HatchIntake::SetIntaking() {
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
    if (m_hatchRollers->GetOutputCurrent() > 20) {
        return true;
    }
    else {
        return false;
    }
}

void HatchIntake::GoToIntakeState(HatchIntake::HatchIntakeState newState) {
    m_hatchIntakeStateTimer = GetMsecTime();
    m_hatchIntakeState = newState;
}

void HatchIntake::GoToPneumaticState(HatchIntake::HatchSolenoidState newState) {
    m_hatchSolenoidStateTimer = GetMsecTime();
    m_hatchSolenoidState = newState;
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
            }
            break;
        case HatchIntakeState::hold:
            m_hatchRollers->Set(ControlMode::PercentOutput, -0.05);
            break;
        case HatchIntakeState::exhaust:
            m_hatchRollers->Set(ControlMode::PercentOutput, 1.0);
            break;
        case HatchIntakeState::launch:
            m_hatchRollers->Set(ControlMode::PercentOutput, 1.0);
            GoToPneumaticState(HatchSolenoidState::launch);
            if ((m_hatchIntakeStateTimer - GetMsecTime()) > 500) {
                GoToIntakeState(HatchIntakeState::idle);
            }
            break;
        case HatchIntakeState::manual:
            // Do Nothing
            break;
    }

    switch (m_hatchSolenoidState) {
        case HatchSolenoidState::launch:
            if (m_hatchSolenoidStateTimer - GetMsecTime() >= 100) {
                m_hatchPuncher->Set(HatchPuncherSolenoidStates::punch);
            }
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
