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
        , m_hatchPneumaticState(HatchPneumaticState::grab) {
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

void HatchIntake::Exhast() {
    GoToIntakeState(HatchIntakeState::exhaust);
}

void HatchIntake::OpenClaw() {
    GoToPneumaticState(HatchPneumaticState::release);
}

void HatchIntake::GrabHatch() {
    GoToPneumaticState(HatchPneumaticState::grab);
}

void HatchIntake::LaunchHatch() {
    GoToPneumaticState(HatchPneumaticState::launch);
}

void HatchIntake::ManualPuncherRetract() {
    GoToPneumaticState(HatchPneumaticState::manual);
    m_hatchPuncher->Set(true);
}

void HatchIntake::GoToIntakeState(HatchIntake::HatchIntakeState newState) {
    m_hatchIntakeState = newState;
}

void HatchIntake::GoToPneumaticState(
    HatchIntake::HatchPneumaticState newState) {
    m_hatchPneumaticStateTimer = GetMsecTime();
    m_hatchPneumaticState = newState;
}

void HatchIntake::TaskPeriodic(RobotMode mode) {
    switch (m_hatchIntakeState) {
        case HatchIntakeState::idle:
            GrabHatch();
            break;
        case HatchIntakeState::intaking:
            OpenClaw();
            if (m_leftHatchSensor->Get() && m_rightHatchSensor->Get()) {
                HoldHatch();
            }
            break;
        case HatchIntakeState::hold:
            GrabHatch();
            break;
        case HatchIntakeState::exhaust:
            LaunchHatch();
            if (m_leftHatchSensor->Get() && m_rightHatchSensor->Get()) {
                HoldHatch();
            }
            else {
                SetIdle();
            }
            break;
        case HatchIntakeState::manual:
            // Do nothing
            break;
    }

    switch (m_hatchPneumaticState) {
        case HatchPneumaticState::release:
            m_hatchClaw->Set(false);
            m_hatchPuncher->Set(false);
            break;
        case HatchPneumaticState::grab:
            m_hatchClaw->Set(true);
            m_hatchPuncher->Set(false);
            break;
        case HatchPneumaticState::launch:
            m_hatchClaw->Set(true);
            if (m_hatchPneumaticStateTimer - GetMsecTime() >= 100) {
                m_hatchPuncher->Set(true);
            }
            break;
        case HatchPneumaticState::manualPunch:
            m_hatchPuncher->Set(true);
            break;
        case HatchPneumaticState::manualRetract:
            m_hatchPuncher->Set(false);
            break;
        case HatchPneumaticState::manual:
            // Do nothing
            break;
    }
}
}
