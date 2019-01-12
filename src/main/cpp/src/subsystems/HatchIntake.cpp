#include "src/subsystems/HatchIntake.h"
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/util/WrapDash.h"

namespace frc973 {
HatchIntake::HatchIntake(TaskMgr *scheduler, LogSpreadsheet *logger,
                         DigitalInput *rightHatchSensor,
                         DigitalInput *leftHatchSensor, Solenoid *hatchClamp,
                         Solenoid *hatchRelease)
        : m_scheduler(scheduler)
        , m_rightHatchSensor(rightHatchSensor)
        , m_leftHatchSensor(leftHatchSensor)
        , m_logger(logger)
        , m_hatchPuncher(new Solenoid(PCM_CAN_ID, HATCH_PUNCHER_PCM_ID))
        , m_hatchClaw(new Solenoid(PCM_CAN_ID, HATCH_CLAW_PCM_ID))
        , m_hatchIntakeState(HatchIntakeState::close) {
    this->m_scheduler->RegisterTask("HatchIntake", this, TASK_PERIODIC);
}

HatchIntake::~HatchIntake() {
    m_scheduler->UnregisterTask(this);
}

void HatchIntake::HatchOpen() {
    GoToState(release);
}

void HatchIntake::HatchGrab() {
    GoToState(grab);
}

void HatchIntake::HatchPush() {
    GoToState(pushOpen);
}

void HatchIntake::HatchPuncherOn() {
    m_hatchPuncher->Set(active);
}

void HatchIntake::HatchPuncherOff() {
    m_hatchPuncher->Set(puncherIdle);
}

void HatchIntake::HatchLaunch() {
    GoToState(preLaunch);
}

void HatchIntake::ClawClose() {
    m_hatchClaw->Set(clawClosed);
}

void HatchIntake::ClawOpen() {
    m_hatchClaw->Set(clawOpen);
}
void HatchIntake::ManualPuncherActivate() {
    GoToState(manual);
    m_hatchPuncher->Set(active);
}

void HatchIntake::ManualPuncherIdle() {
    GoToState(manual);
    m_hatchPuncher->Set(puncherIdle);
}

void HatchIntake::GoToState(HatchIntakeState newState) {
    m_stateStartTimeMs = GetMsecTime();
    m_hatchIntakeState = newState;
    switch (m_hatchIntakeState) {
        case HatchIntakeState::release:
            m_hatchClaw->Set(clawOpen);
            m_hatchPuncher->Set(puncherIdle);
            break;
        case HatchIntakeState::grab:
            m_hatchClaw->Set(clawClosed);
            m_hatchPuncher->Set(puncherIdle);
            break;
        case HatchIntakeState::pushOpen:
            m_hatchClaw->Set(clawOpen);
            m_hatchPuncher->Set(active);
            break;
        case HatchIntakeState::pushClose:
            m_hatchClaw->Set(clawClosed);
            m_hatchPuncher->Set(puncherIdle);
            break;
        case HatchIntakeState::preLaunch:
            m_hatchClaw->Set(clawClosed);
            m_hatchPuncher->Set(active);
            break;
        case HatchIntakeState::launch:
            m_hatchClaw->Set(clawOpen);
            break;
        case HatchIntakeState::launchReset:
            m_hatchClaw->Set(clawOpen);
            m_hatchPuncher->Set(puncherIdle);
            break;
        case HatchIntakeState::manual:
            break;
    }
}
}
