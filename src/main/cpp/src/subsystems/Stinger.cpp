#include "src/subsystems/Stinger.h"

namespace frc973 {
Stinger::Stinger(TaskMgr *scheduler, LogSpreadsheet *logger,
                 GreyTalonSRX *stingerElevatorMotor,
                 GreyTalonSRX *stingerDriveMotor,
                 DigitalInput *stingerLowerHall, DigitalInput *stingerUpperHall)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_stingerElevatorMotor(stingerElevatorMotor)
        , m_stingerDriveMotor(stingerDriveMotor)
        , m_stingerLowerHall(stingerLowerHall)
        , m_stingerUpperHall(stingerUpperHall)
        , m_stingerState(StingerState::manualVoltage) {
    this->m_scheduler->RegisterTask("Stinger", this, TASK_PERIODIC);
    m_stingerElevatorMotor->ConfigSelectedFeedbackSensor(
        ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0,
        10);  // 0 = Not cascaded PID Loop; 10 = in constructor, not in a loop
    m_stingerElevatorMotor->SetSensorPhase(true);
    m_stingerElevatorMotor->SetNeutralMode(NeutralMode::Coast);
    m_stingerElevatorMotor->SetInverted(true);

    m_stingerElevatorMotor->Config_PID(0, 0.0, 0.0, 0.0, 0.0, 10);
    m_stingerElevatorMotor->ConfigMotionCruiseVelocity(3750.0, 10);
    m_stingerElevatorMotor->ConfigMotionAcceleration(4200.0, 10);
    m_stingerElevatorMotor->SelectProfileSlot(0, 0);

    m_stingerElevatorMotor->EnableCurrentLimit(true);
    m_stingerElevatorMotor->ConfigPeakCurrentDuration(0, 10);
    m_stingerElevatorMotor->ConfigPeakCurrentLimit(0, 10);
    m_stingerElevatorMotor->ConfigContinuousCurrentLimit(50, 10);
    m_stingerElevatorMotor->EnableVoltageCompensation(false);
    //  m_stingerElevatorMotor->ConfigForwardSoftLimitThreshold(
    //       STINGER_SOFT_HEIGHT_LIMIT / STINGER_INCHES_PER_CLICK, 10);
    // m_stingerElevatorMotor->ConfigForwardSoftLimitEnable(true, 10);

    m_stingerElevatorMotor->Set(ControlMode::PercentOutput, 0.0);

    m_current = new LogCell("Stinger Current", 32, true);
    m_logger->RegisterCell(m_current);

    m_positionCell = new LogCell("Stinger Position", 32, true);
    m_logger->RegisterCell(m_positionCell);
}

Stinger::~Stinger() {
    m_scheduler->UnregisterTask(this);
}

void Stinger::SetPower(double power) {
    GoToStingerState(StingerState::manualVoltage);
    power = Util::bound(power, -1.0, 1.0);
    if (power > 0.0 &&
        GetStingerElevatorHallState() == StingerElevatorHallState::top) {
        m_stingerElevatorMotor->Set(ControlMode::PercentOutput, 0.0);
    }
    if (power < 0.0 &&
        GetStingerElevatorHallState() == StingerElevatorHallState::bottom) {
        m_stingerElevatorMotor->Set(ControlMode::PercentOutput, 0.0);
    }
    if (GetStingerElevatorHallState() == StingerElevatorHallState::middle) {
        m_stingerElevatorMotor->Set(ControlMode::PercentOutput, power);
    }
}

void Stinger::SetPositionInches(double position) {
    GoToStingerState(StingerState::motionMagic);
    int position_clicks = position / STINGER_INCHES_PER_CLICK;
    m_stingerElevatorMotor->Set(ControlMode::MotionMagic, position_clicks);
}

double Stinger::GetPositionInches() {
    return STINGER_INCHES_PER_CLICK *
           m_stingerElevatorMotor->GetSelectedSensorPosition(0);
}

void Stinger::Stow() {
    SetPositionInches(STOW);
}

void Stinger::SetMiddle() {
    SetPositionInches(MIDDLE);
}

void Stinger::Deploy() {
    SetPositionInches(BOTTOM);
}

bool Stinger::GetLowerHall() {
    return m_stingerLowerHall->Get();
}

bool Stinger::GetUpperHall() {
    return m_stingerUpperHall->Get();
}

Stinger::StingerElevatorHallState Stinger::GetStingerElevatorHallState() {
    // Elevator at top
    if (!Stinger::GetLowerHall() && Stinger::GetUpperHall()) {
        return StingerElevatorHallState::top;
    }

    // Elevator at middle
    if (!Stinger::GetLowerHall() && !Stinger::GetUpperHall()) {
        return StingerElevatorHallState::middle;
    }

    // Elevator at bottom
    if (Stinger::GetLowerHall() && !Stinger::GetUpperHall()) {
        return StingerElevatorHallState::bottom;
    }

    // Elevator triggering both halls, error, check polarity
    if (Stinger::GetLowerHall() && Stinger::GetUpperHall()) {
        return StingerElevatorHallState::error;
    }
}

void Stinger::GoToStingerState(Stinger::StingerState newState) {
    m_stingerState = newState;
}

void Stinger::TaskPeriodic(RobotMode mode) {
    m_current->LogDouble(m_stingerElevatorMotor->GetOutputCurrent());
    m_positionCell->LogDouble(GetPositionInches());
    SmartDashboard::PutNumber("stinger/encoders/encoder", GetPositionInches());
    SmartDashboard::PutNumber("stinger/outputs/current",
                              m_stingerElevatorMotor->GetOutputCurrent());

    switch (m_stingerState) {
        case StingerState::manualVoltage:
            break;
        case StingerState::motionMagic:
            break;
    }
}
}