#include "src/subsystems/CargoIntake.h"

namespace frc973 {
CargoIntake::CargoIntake(TaskMgr *scheduler, LogSpreadsheet *logger,
                         GreyTalonSRX *cargoIntakeMotor,
                         Solenoid *cargoWristLock, Solenoid *cargoWrist,
                         Solenoid *cargoWheelPiston)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_cargoIntakeMotor(cargoIntakeMotor)
        , m_cargoWrist(cargoWrist)
        , m_cargoWristLock(cargoWristLock)
        , m_cargoWheelPiston(cargoWheelPiston)
        , m_cargoIntakeState(CargoIntakeState::notRunning)
        , m_cargoWristLockState(CargoWristLockState::unlocked)
        , m_cargoWristState(CargoWristState::retracted)
        , m_cargoWheelPistonState(CargoWheelPistonState::retracted)
        , m_cargoEndgameState(CargoEndgameState::notEndgame) {
    this->m_scheduler->RegisterTask("CargoIntake", this, TASK_PERIODIC);
    m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 0.0);
    m_cargoIntakeMotor->SetNeutralMode(NeutralMode::Coast);
    m_cargoIntakeMotor->EnableCurrentLimit(false);

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

void CargoIntake::DeployWheelPiston() {
    GoToWheelPistonState(CargoWheelPistonState::deployed);
}

void CargoIntake::RetractWheelPiston() {
    GoToWheelPistonState(CargoWheelPistonState::retracted);
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

CargoIntake::CargoWheelPistonState CargoIntake::GetWheelPistonState() {
    return m_cargoWheelPistonState;
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

void CargoIntake::GoToWheelPistonState(
    CargoIntake::CargoWheelPistonState newState) {
    m_cargoWheelPistonState = newState;
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
            break;
        case CargoIntakeState::holding:
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 0.2);
            break;
        case CargoIntakeState::notRunning:
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 0.0);
            break;
        case CargoIntakeState::reverse:
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, -1.0);
            break;
    }

    switch (m_cargoWristLockState) {
        case CargoWristLockState::unlocked:
            m_cargoWristLock->Set(false);
            break;
        case CargoWristLockState::locked:
            m_cargoWristLock->Set(true);
            break;
    }

    switch (m_cargoWristState) {
        case CargoWristState::extended:
            m_cargoWrist->Set(true);
            break;
        case CargoWristState::retracted:
            m_cargoWrist->Set(false);
            break;
    }

    switch (m_cargoWheelPistonState) {
        case CargoWheelPistonState::retracted:
            m_cargoWheelPiston->Set(false);
            break;
        case CargoWheelPistonState::deployed:
            m_cargoWheelPiston->Set(true);
            break;
    }

    switch (m_cargoEndgameState) {
        case CargoEndgameState::notEndgame:
            // Do nothing
            break;
        case CargoEndgameState::stowed:
            RetractWheelPiston();
            UnlockWrist();
            if (m_cargoEndgameTimer - GetMsecTime() >= 100) {
                RetractWrist();
            }
            break;
        case CargoEndgameState::deployed:
            ExtendWrist();
            if (m_cargoEndgameTimer - GetMsecTime() >= 100) {
                LockWrist();
                DeployWheelPiston();
            }
            break;
    }
}
}
