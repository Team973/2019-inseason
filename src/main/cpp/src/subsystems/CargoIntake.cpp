#include "src/subsystems/CargoIntake.h"

namespace frc973 {
CargoIntake::CargoIntake(TaskMgr *scheduler, LogSpreadsheet *logger,
                         GreyTalonSRX *cargoIntakeMotor,
                         Solenoid *cargoWristLock, Solenoid *cargoWrist,
                         Solenoid *cargoPlatformWheel)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_cargoIntakeMotor(cargoIntakeMotor)
        , m_cargoWrist(cargoWrist)
        , m_cargoWristLock(cargoWristLock)
        , m_cargoPlatformWheel(cargoPlatformWheel)
        , m_cargoIntakeState(CargoIntakeState::notRunning)
        , m_cargoWristLockState(CargoWristLockState::unlocked)
        , m_cargoWristLockPneumaticState(CargoWristLockPneumaticState::unlocked)
        , m_cargoWristState(CargoWristState::retracted)
        , m_cargoWristPneumaticState(CargoWristPneumaticState::retracted)
        , m_cargoPlatformWheelState(CargoPlatformWheelState::retracted)
        , m_cargoPlatfromWheelPneumaticState(
              CargoPlatformWheelPneumaticState::retracted)
        , m_cargoEndgameState(CargoEndgameState::notEndgame) {
    this->m_scheduler->RegisterTask("CargoIntake", this, TASK_PERIODIC);
    m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 0.0);
    m_cargoIntakeMotor->SetNeutralMode(NeutralMode::Coast);

    m_cargoIntakeMotor->EnableCurrentLimit(true);
    m_cargoIntakeMotor->ConfigPeakCurrentDuration(0, 10);
    m_cargoIntakeMotor->ConfigPeakCurrentLimit(0, 10);
    m_cargoIntakeMotor->ConfigContinuousCurrentLimit(40, 10);
    m_cargoIntakeMotor->EnableVoltageCompensation(false);

    m_current = new LogCell("CargoIntake Current", 32, true);
    m_logger->RegisterCell(m_current);
}

CargoIntake::~CargoIntake() {
    m_scheduler->UnregisterTask(this);
}

void CargoIntake::RunIntake() {
    GoToIntakeState(CargoIntakeState::running);
}

void CargoIntake::HoldCargo() {
    GoToIntakeState(CargoIntakeState::holding);
}

void CargoIntake::StopIntake() {
    GoToIntakeState(CargoIntakeState::notRunning);
}

void CargoIntake::Exhaust() {
    GoToIntakeState(CargoIntakeState::reverse);
}

void CargoIntake::UnlockWrist() {
    GoToWristLockState(CargoWristLockState::unlocked);
}

void CargoIntake::LockWrist() {
    GoToWristLockState(CargoWristLockState::locked);
}

void CargoIntake::ExtendWrist() {
    GoToWristState(CargoWristState::extended);
}

void CargoIntake::RetractWrist() {
    GoToWristState(CargoWristState::retracted);
}

void CargoIntake::DeployPlatformWheel() {
    GoToPlatformWheelState(CargoPlatformWheelState::deployed);
}

void CargoIntake::RetractPlatformWheel() {
    GoToPlatformWheelState(CargoPlatformWheelState::retracted);
}

double CargoIntake::GetIntakeCurrent() {
    return m_cargoIntakeMotor->GetOutputCurrent();
}

CargoIntake::CargoIntakeState CargoIntake::GetIntakeState() {
    return m_cargoIntakeState;
}

CargoIntake::CargoWristLockState CargoIntake::GetWristLockState() {
    return m_cargoWristLockState;
}

CargoIntake::CargoWristState CargoIntake::GetWristState() {
    return m_cargoWristState;
}

CargoIntake::CargoPlatformWheelState CargoIntake::GetPlatformWheelState() {
    return m_cargoPlatformWheelState;
}

void CargoIntake::GoToIntakeState(CargoIntake::CargoIntakeState newState) {
    m_cargoIntakeState = newState;
}

void CargoIntake::GoToWristLockState(
    CargoIntake::CargoWristLockState newState) {
    m_cargoWristLockState = newState;
}

void CargoIntake::GoToWristState(CargoIntake::CargoWristState newState) {
    m_cargoWristState = newState;
}

void CargoIntake::GoToPlatformWheelState(
    CargoIntake::CargoPlatformWheelState newState) {
    m_cargoPlatformWheelState = newState;
}

void CargoIntake::GoToEndgameState(CargoIntake::CargoEndgameState newState) {
    m_cargoEndgameTimer = GetMsecTime();
    m_cargoEndgameState = newState;
}

void CargoIntake::TaskPeriodic(RobotMode mode) {
    m_current->LogDouble(m_cargoIntakeMotor->GetOutputCurrent());

    switch (m_cargoIntakeState) {
        case CargoIntakeState::running:
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 1.0);
            if (m_cargoIntakeMotor->GetOutputCurrent() > 20) {
                m_cargoIntakeState = CargoIntakeState::holding;
            }
            break;
        case CargoIntakeState::holding:
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 0.2);
            this->RetractWrist();
            break;
        case CargoIntakeState::notRunning:
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 0.0);
            break;
        case CargoIntakeState::reverse:
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, -1.0);
            this->ExtendWrist();
            break;
    }

    switch (m_cargoWristLockState) {
        case CargoWristLockState::unlocked:
            m_cargoWristLockPneumaticState =
                CargoWristLockPneumaticState::unlocked;
            break;
        case CargoWristLockState::locked:
            m_cargoWristLockPneumaticState =
                CargoWristLockPneumaticState::locked;
            break;
    }

    switch (m_cargoWristLockPneumaticState) {
        case CargoWristLockPneumaticState::unlocked:
            m_cargoWristLock->Set(false);
            break;
        case CargoWristLockPneumaticState::locked:
            m_cargoWristLock->Set(true);
    }

    switch (m_cargoWristState) {
        case CargoWristState::extended:
            m_cargoWristPneumaticState = CargoWristPneumaticState::extended;
            break;
        case CargoWristState::retracted:
            m_cargoWristPneumaticState = CargoWristPneumaticState::retracted;
            break;
    }

    switch (m_cargoWristPneumaticState) {
        case CargoWristPneumaticState::extended:
            m_cargoWrist->Set(true);
            break;
        case CargoWristPneumaticState::retracted:
            m_cargoWrist->Set(false);
            break;
    }

    switch (m_cargoPlatformWheelState) {
        case CargoPlatformWheelState::retracted:
            m_cargoPlatfromWheelPneumaticState =
                CargoPlatformWheelPneumaticState::retracted;
            break;
        case CargoPlatformWheelState::deployed:
            m_cargoPlatfromWheelPneumaticState =
                CargoPlatformWheelPneumaticState::deployed;
            break;
    }

    switch (m_cargoPlatfromWheelPneumaticState) {
        case CargoPlatformWheelPneumaticState::retracted:
            m_cargoPlatformWheel->Set(false);
            break;
        case CargoPlatformWheelPneumaticState::deployed:
            m_cargoPlatformWheel->Set(true);
            break;
    }

    switch (m_cargoEndgameState) {
        case CargoEndgameState::notEndgame:
            // Do nothing
            break;
        case CargoEndgameState::stowed:
            RetractPlatformWheel();
            UnlockWrist();
            if (m_cargoEndgameTimer - GetMsecTime() >= 100) {
                RetractWrist();
            }
            break;
        case CargoEndgameState::deployed:
            ExtendWrist();
            if (m_cargoEndgameTimer - GetMsecTime() >= 100) {
                LockWrist();
                DeployPlatformWheel();
            }
            break;
    }
}
}
