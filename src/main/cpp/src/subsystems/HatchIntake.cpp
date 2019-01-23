#include "src/subsystems/HatchIntake.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/util/WrapDash.h"

namespace frc973 {
HatchIntake::HatchIntake(TaskMgr *scheduler, LogSpreadsheet *logger,
                         Solenoid *hatchClaw, Solenoid *hatchPuncher,
                         DigitalInput *leftHatchSensor,
                         DigitalInput *rightHatchSensor)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_hatchClaw(hatchClaw)
        , m_hatchPuncher(hatchPuncher)
        , m_leftHatchSensor(leftHatchSensor)
        , m_rightHatchSensor(rightHatchSensor)
        , m_hatchIntakeState(HatchIntakeState::idle)
        , m_hatchSolenoidState(HatchSolenoidState::grab) {
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

void HatchIntake::OpenClaw() {
    GoToPneumaticState(HatchSolenoidState::release);
    GoToIntakeState(HatchIntakeState::manual);
}

void HatchIntake::LaunchHatch() {
    GoToPneumaticState(HatchSolenoidState::launch);
}

void HatchIntake::GrabHatch() {
    GoToPneumaticState(HatchSolenoidState::grab);
    GoToIntakeState(HatchIntakeState::manual);
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
    return (m_leftHatchSensor->Get() && m_rightHatchSensor->Get());
}

void HatchIntake::GoToIntakeState(HatchIntake::HatchIntakeState newState) {
    m_hatchIntakeState = newState;
}

void HatchIntake::GoToPneumaticState(HatchIntake::HatchSolenoidState newState) {
    m_hatchSolenoidStateTimer = GetMsecTime();
    m_hatchSolenoidState = newState;
}

void HatchIntake::TaskPeriodic(RobotMode mode) {
    switch (m_hatchIntakeState) {
        case HatchIntakeState::idle:
            GrabHatch();
            break;
        case HatchIntakeState::intaking:
            OpenClaw();
            if (IsHatchInIntake()) {
                HoldHatch();
            }
            break;
        case HatchIntakeState::hold:
            GrabHatch();
            break;
        case HatchIntakeState::exhaust:
            if (IsHatchInIntake()) {
                LaunchHatch();
            }
            else {
                SetIdle();
            }
            break;
        case HatchIntakeState::manual:
            // Do nothing
            break;
    }

    switch (m_hatchSolenoidState) {
        case HatchSolenoidState::release:
            m_hatchClaw->Set(HatchClawSolenoidStates::release);
            m_hatchPuncher->Set(HatchPuncherSolenoidStates::retract);
            break;
        case HatchSolenoidState::grab:
            m_hatchClaw->Set(HatchClawSolenoidStates::grab);
            m_hatchPuncher->Set(HatchPuncherSolenoidStates::retract);
            break;
        case HatchSolenoidState::launch:
            m_hatchClaw->Set(HatchClawSolenoidStates::grab);
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
