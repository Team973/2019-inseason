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
        , m_kickOffPneumatic(
              new DoubleSolenoid(PCM_CAN_ID, KICKOFF_FORWARD, KICKOFF_REVERSE))
        , m_sneakyClimb(new DoubleSolenoid(PCM_CAN_ID, SNEAKY_CLIMB_FORWARD,
                                           SNEAKY_CLIMB_REVERSE))
        , m_power(0.0)
        , m_stingerState(StingerState::manualVoltage) {
    this->m_scheduler->RegisterTask("Stinger", this, TASK_PERIODIC);
    m_stingerElevatorMotor->SetSensorPhase(true);
    m_stingerElevatorMotor->SetNeutralMode(NeutralMode::Brake);
    m_stingerElevatorMotor->SetInverted(true);

    m_stingerElevatorMotor->Config_PID(0, 0.0, 0.0, 0.0, 0.0, 10);
    m_stingerElevatorMotor->ConfigMotionCruiseVelocity(3750.0, 10);
    m_stingerElevatorMotor->ConfigMotionAcceleration(4200.0, 10);
    m_stingerElevatorMotor->SelectProfileSlot(0, 0);

    m_stingerElevatorMotor->EnableCurrentLimit(true);
    m_stingerElevatorMotor->ConfigPeakCurrentDuration(0, 10);
    m_stingerElevatorMotor->ConfigPeakCurrentLimit(0, 10);
    m_stingerElevatorMotor->ConfigContinuousCurrentLimit(100, 10);
    m_stingerElevatorMotor->EnableVoltageCompensation(false);

    m_stingerElevatorMotor->Set(ControlMode::PercentOutput, 0.0);

    m_kickOffPneumatic->Set(DoubleSolenoid::Value::kReverse);
    m_sneakyClimb->Set(DoubleSolenoid::Value::kReverse);

    m_current = new LogCell("Stinger Current", 32, true);
    m_logger->RegisterCell(m_current);
}

Stinger::~Stinger() {
    m_scheduler->UnregisterTask(this);
}

void Stinger::SetPower(double power) {  // negative sets stinger down, brings
                                        // robot up and vice versa
    power = Util::bound(power, -1.0, 1.0);
    m_power = power;
    GoToStingerState(StingerState::manualVoltage);
}

void Stinger::SetPositionInches(double position) {
    GoToStingerState(StingerState::motionMagic);
    int position_clicks = position / STINGER_INCHES_PER_CLICK;
    m_stingerElevatorMotor->Set(ControlMode::MotionMagic, position_clicks);
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
    return !m_stingerLowerHall->Get();
}

bool Stinger::GetUpperHall() {
    return !m_stingerUpperHall->Get();
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

void Stinger::SetKickUpEnable() {
    m_kickOffPneumatic->Set(DoubleSolenoid::Value::kForward);
}

void Stinger::SetKickUpDisable() {
    m_kickOffPneumatic->Set(DoubleSolenoid::Value::kReverse);
}

void Stinger::DeploySwitchBlade() {
    m_sneakyClimb->Set(DoubleSolenoid::Value::kForward);
}

void Stinger::RetractSwitchBlade() {
    m_sneakyClimb->Set(DoubleSolenoid::Value::kReverse);
}

void Stinger::TaskPeriodic(RobotMode mode) {
    m_current->LogDouble(m_stingerElevatorMotor->GetOutputCurrent());

    DBStringPrintf(DBStringPos::DB_LINE6, "pldrivec:%2.2lf",
                   m_stingerDriveMotor->GetOutputCurrent());
    switch (m_stingerState) {
        case StingerState::manualVoltage:
            if (m_power > 0.0 && GetStingerElevatorHallState() ==
                                     StingerElevatorHallState::top) {
                m_stingerElevatorMotor->Set(ControlMode::PercentOutput, 0.0);
            }
            else if (m_power < 0.0 && GetStingerElevatorHallState() ==
                                          StingerElevatorHallState::bottom) {
                m_stingerElevatorMotor->Set(ControlMode::PercentOutput,
                                            fmax(-0.3, m_power));
            }
            else {
                m_stingerElevatorMotor->Set(ControlMode::PercentOutput,
                                            m_power);
            }
            break;
        case StingerState::motionMagic:
            break;
        case StingerState::idle:
            m_stingerElevatorMotor->Set(ControlMode::PercentOutput, 0.0);
            break;
    }
}
}
