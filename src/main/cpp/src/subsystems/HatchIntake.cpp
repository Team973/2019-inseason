#include "src/subsystems/HatchIntake.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/util/WrapDash.h"

namespace frc973 {
HatchIntake::HatchIntake(TaskMgr *scheduler, LogSpreadsheet *logger)
        : m_scheduler(scheduler)
        , m_rightHatchSensor(new DigitalInput(RIGHT_HATCH_SENSOR_ID))
        , m_leftHatchSensor(new DigitalInput(LEFT_HATCH_SENSOR_ID))
        , m_logger(logger)
        , m_hatchPuncher(new Solenoid(PCM_CAN_ID, HATCH_PUNCHER_PCM_ID))
        , m_hatchClaw(new Solenoid(PCM_CAN_ID, HATCH_CLAW_PCM_ID))
        , m_hatchIntakeState(HatchIntakeState::idle) {
    this->m_scheduler->RegisterTask("HatchIntake", this, TASK_PERIODIC);
}

HatchIntake::~HatchIntake() {
    m_scheduler->UnregisterTask(this);
}

void HatchIntake::HatchOpen() {
    GoToState(PneumaticState::release);
}

void HatchIntake::HatchGrab() {
    GoToState(PneumaticState::grab);
}

void HatchIntake::HatchPush() {
    GoToState(PneumaticState::pushOpen);
}

void HatchIntake::HatchPuncherOn() {
    m_hatchPuncher->Set(active);
}

void HatchIntake::HatchLaunch() {
    GoToState(PneumaticState::launch);
}

void HatchIntake::ClawClose() {
    m_hatchClaw->Set(clawClosed);
}

void HatchIntake::ClawOpen() {
    m_hatchClaw->Set(clawOpen);
}
void HatchIntake::ManualPuncherActivate() {
    GoToState(PneumaticState::manual);
    m_hatchPuncher->Set(active);
}

void HatchIntake::SetIntakeState(HatchIntakeState state) {
    m_hatchIntakeState = state;
}

void HatchIntake::TaskPeriodic(RobotMode mode) {
    switch (m_hatchIntakeState) {
        case HatchIntakeState::idle:
            GoToState(PneumaticState::grab);
            break;
        case HatchIntakeState::intaking:
            GoToState(PneumaticState::release);
            if (m_leftHatchSensor->Get() && m_rightHatchSensor->Get()) {
                m_hatchIntakeState = HatchIntakeState::hold;
            }
            break;
        case HatchIntakeState::hold:
            GoToState(PneumaticState::grab);
            break;
        case HatchIntakeState::exhaust:
            GoToState(PneumaticState::launch);
            if (m_leftHatchSensor->Get() && m_rightHatchSensor->Get()) {
                m_hatchIntakeState = HatchIntakeState::hold;
            }
            else {
                m_hatchIntakeState = HatchIntakeState::idle;
            }
            break;
    }
}

void HatchIntake::GoToState(PneumaticState state) {
    m_stateStartTimeMs = GetMsecTime();
    switch (state) {
        case PneumaticState::release:
            m_hatchClaw->Set(clawOpen);
            m_hatchPuncher->Set(puncherIdle);
            break;
        case PneumaticState::grab:
            m_hatchClaw->Set(clawClosed);
            m_hatchPuncher->Set(puncherIdle);
            break;
        case PneumaticState::pushOpen:
            m_hatchClaw->Set(clawOpen);
            m_hatchPuncher->Set(active);
            break;
        case PneumaticState::pushClose:
            m_hatchClaw->Set(clawClosed);
            m_hatchPuncher->Set(puncherIdle);
            break;
        case PneumaticState::launch:
            m_hatchClaw->Set(clawOpen);
            m_hatchPuncher->Set(active);
            break;
        case PneumaticState::manual:
            break;
    }
}
}
