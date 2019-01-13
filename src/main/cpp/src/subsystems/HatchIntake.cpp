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
        , m_hatchIntakeState(HatchIntakeState::grab) {
    this->m_scheduler->RegisterTask("HatchIntake", this, TASK_PERIODIC);
}

HatchIntake::~HatchIntake() {
    m_scheduler->UnregisterTask(this);
}

void HatchIntake::ClawOpen() {
    m_hatchClaw->Set(ClawState::open);
}

void HatchIntake::ClawClose() {
    m_hatchClaw->Set(ClawState::close);
}

void HatchIntake::PuncherOn() {
    m_hatchPuncher->Set(PuncherState::punch);
}

void HatchIntake::PuncherOff() {
    m_hatchPuncher->Set(PuncherState::retract);
}

void HatchIntake::HatchOpen() {
    m_hatchIntakeState = HatchIntakeState::release;
}

void HatchIntake::HatchGrab() {
    m_hatchIntakeState = HatchIntakeState::grab;
}

void HatchIntake::HatchPush() {
    m_hatchIntakeState = HatchIntakeState::pushOpen;
}

void HatchIntake::HatchLaunch() {
    m_hatchIntakeState = HatchIntakeState::preLaunch;
}

void HatchIntake::ManualClawOpen() {
    m_hatchIntakeState = HatchIntakeState::manual;
    ClawOpen();
}

void HatchIntake::ManualClawClose() {
    m_hatchIntakeState = HatchIntakeState::manual;
    ClawClose();
}

void HatchIntake::ManualPuncherOn() {
    m_hatchIntakeState = HatchIntakeState::manual;
    PuncherOn();
}

void HatchIntake::ManualPuncherOff() {
    m_hatchIntakeState = HatchIntakeState::manual;
    PuncherOff();
}

void HatchIntake::TaskPeriodic(RobotMode mode) {
    switch (m_hatchIntakeState) {
        case HatchIntakeState::release:
            ClawOpen();
            PuncherOff();
            break;
        case HatchIntakeState::grab:
            ClawClose();
            PuncherOff();
            break;
        case HatchIntakeState::pushOpen:
            if (GetMsecTime() - m_stateStartTimeMs > 1000) {
                ClawOpen();
                PuncherOn();
            }
            break;
        case HatchIntakeState::pushClose:
            ClawClose();
            PuncherOff();
            break;
        case HatchIntakeState::preLaunch:
            if (GetMsecTime() - m_stateStartTimeMs > 100) {
                ClawClose();
                PuncherOn();
                m_hatchIntakeState = HatchIntakeState::launch;
            }
            break;
        case HatchIntakeState::launch:
            if (GetMsecTime() - m_stateStartTimeMs > 500) {
                ClawOpen();
            }
            break;
        case HatchIntakeState::launchReset:
            ClawOpen();
            PuncherOff();
            break;
        case HatchIntakeState::manual:
            // manual control
            break;
    }
}
}
