#include "src/subsystems/CargoIntake.h"
#include "lib/util/WrapDash.h"

namespace frc973 {
CargoIntake::CargoIntake(TaskMgr *scheduler, LogSpreadsheet *logger,
                         GreyTalonSRX *cargoIntakeMotor,
                         Solenoid *cargoPlatformLock, Solenoid *cargoWrist,
                         Limelight *limelightHatch)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_cargoIntakeMotor(cargoIntakeMotor)
        , m_cargoPlatformLock(cargoPlatformLock)
        , m_power(0.0)
        , m_cargoWrist(cargoWrist)
        , m_cargoIntakeState(CargoIntakeState::notRunning)
        , m_cargoWristState(CargoWristState::retracted)
        , m_limelightHatch(limelightHatch)
        , m_cargoTimer(0.0)
        , m_cargoPlatformLockState(CargoPlatformLockState::retracted)
        , m_intakeCurentFilter(new MovingAverageFilter(0.9)) {
    this->m_scheduler->RegisterTask("CargoIntake", this, TASK_PERIODIC);
    m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 0.0);
    m_cargoIntakeMotor->SetNeutralMode(NeutralMode::Coast);

    m_cargoIntakeMotor->EnableCurrentLimit(true);
    m_cargoIntakeMotor->ConfigPeakCurrentDuration(0, 10);
    m_cargoIntakeMotor->ConfigPeakCurrentLimit(0, 10);
    m_cargoIntakeMotor->ConfigContinuousCurrentLimit(40, 10);
    m_cargoIntakeMotor->EnableVoltageCompensation(false);
    m_cargoIntakeMotor->SetInverted(true);
    m_cargoIntakeMotor->ConfigNeutralDeadband(0.01);

    m_currentCell = new LogCell("Cargo Current", 32, true);
    m_voltageCell = new LogCell("Cargo Voltage", 32, true);
    logger->RegisterCell(m_currentCell);
    logger->RegisterCell(m_voltageCell);
}

CargoIntake::~CargoIntake() {
    m_scheduler->UnregisterTask(this);
}

void CargoIntake::RunIntake(double power) {
    m_cargoIntakeState = CargoIntakeState::manualRunning;
    m_power = power;
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

void CargoIntake::ExtendWrist() {
    GoToWristState(CargoWristState::extended);
}

void CargoIntake::RetractWrist() {
    GoToWristState(CargoWristState::retracted);
}

void CargoIntake::DeployPlatformWheel() {
    GoToPlatformLockState(CargoPlatformLockState::deployed);
}

void CargoIntake::RetractPlatformWheel() {
    GoToPlatformLockState(CargoPlatformLockState::retracted);
}

double CargoIntake::GetIntakeCurrent() {
    return m_cargoIntakeMotor->GetOutputCurrent();
}

CargoIntake::CargoIntakeState CargoIntake::GetIntakeState() {
    return m_cargoIntakeState;
}

CargoIntake::CargoWristState CargoIntake::GetWristState() {
    return m_cargoWristState;
}

CargoIntake::CargoPlatformLockState CargoIntake::GetPlatformLockState() {
    return m_cargoPlatformLockState;
}

void CargoIntake::GoToIntakeState(CargoIntake::CargoIntakeState newState) {
    m_cargoIntakeState = newState;
}

void CargoIntake::GoToWristState(CargoIntake::CargoWristState newState) {
    m_cargoWristState = newState;
}

void CargoIntake::GoToPlatformLockState(
    CargoIntake::CargoPlatformLockState newState) {
    m_cargoPlatformLockState = newState;
}

void CargoIntake::EnableBrakeMode() {
    m_cargoIntakeMotor->SetNeutralMode(NeutralMode::Brake);
}

void CargoIntake::EnableCoastMode() {
    m_cargoIntakeMotor->SetNeutralMode(NeutralMode::Coast);
}

void CargoIntake::TaskPeriodic(RobotMode mode) {
    m_currentCell->LogDouble(m_cargoIntakeMotor->GetOutputCurrent());
    m_voltageCell->LogDouble(m_cargoIntakeMotor->GetMotorOutputVoltage());
    DBStringPrintf(DBStringPos::DB_LINE6, "cp %2.2lf",
                   m_cargoIntakeMotor->GetOutputCurrent());
    double filteredCurrent =
        m_intakeCurentFilter->Update(m_cargoIntakeMotor->GetOutputCurrent());
    switch (m_cargoIntakeState) {
        case CargoIntakeState::running:
            m_cargoIntakeMotor->ConfigContinuousCurrentLimit(40, 10);
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 1.0);
            ExtendWrist();
            if (filteredCurrent > 25.0) {
                m_cargoIntakeState = CargoIntakeState::holding;
                m_limelightHatch->SetLightBlink();
                this->RetractWrist();
            }
            break;
        case CargoIntakeState::manualRunning:
            RetractWrist();
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, m_power);
            if (filteredCurrent > 25.0) {
                m_cargoIntakeState = CargoIntakeState::holding;
            }
            break;
        case CargoIntakeState::holding:
            m_cargoIntakeMotor->ConfigContinuousCurrentLimit(100, 10);
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 0.7);
            break;
        case CargoIntakeState::notRunning:
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 0.0);
            break;
        case CargoIntakeState::reverse:
            m_cargoIntakeMotor->ConfigContinuousCurrentLimit(40, 10);
            m_cargoIntakeMotor->Set(ControlMode::PercentOutput, -1.0);
            this->RetractWrist();
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

    switch (m_cargoPlatformLockState) {
        case CargoPlatformLockState::retracted:
            m_cargoPlatformLock->Set(false);
            break;
        case CargoPlatformLockState::deployed:
            m_cargoPlatformLock->Set(true);
            break;
    }
}
}
