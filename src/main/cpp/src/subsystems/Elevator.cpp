#include "src/subsystems/Elevator.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/util/WrapDash.h"

using namespace frc;

namespace frc973 {
Elevator::Elevator(TaskMgr *scheduler, LogSpreadsheet *logger,
                   GreyTalonSRX *elevatorMotorA, VictorSPX *elevatorMotorB,
                   ObservableXboxJoystick *operatorJoystick,
                   DigitalInput *elevatorHall)
        : m_scheduler(scheduler)
        , m_elevatorMotorA(elevatorMotorA)
        , m_elevatorMotorB(elevatorMotorB)
        , m_operatorJoystick(operatorJoystick)
        , m_elevatorHall(elevatorHall)
        , m_position(0.0)
        , m_joystickControl(0.0)
        , m_prevHall(true)
        , m_zeroingTime(0)
        , m_elevatorState(ElevatorState::idle) {
    this->m_scheduler->RegisterTask("Elevator", this, TASK_PERIODIC);

    m_elevatorMotorA->ConfigSelectedFeedbackSensor(
        ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0,
        10);  // 0 = Not cascaded PID Loop; 10 = in constructor, not in a loop
    m_elevatorMotorA->SetSensorPhase(false);
    m_elevatorMotorA->SetNeutralMode(NeutralMode::Coast);
    m_elevatorMotorA->SetInverted(false);

    m_elevatorMotorA->Config_PID(0, 1.5, 0.0, 0.0, 0.0, 10);
    m_elevatorMotorA->ConfigMotionCruiseVelocity(5000.0, 10);
    m_elevatorMotorA->ConfigMotionAcceleration(6000.0, 10);
    m_elevatorMotorA->SelectProfileSlot(0, 0);

    m_elevatorMotorA->EnableCurrentLimit(true);
    m_elevatorMotorA->ConfigPeakCurrentDuration(0, 10);
    m_elevatorMotorA->ConfigPeakCurrentLimit(0, 10);
    m_elevatorMotorA->ConfigContinuousCurrentLimit(60, 10);
    m_elevatorMotorA->EnableVoltageCompensation(false);
    m_elevatorMotorA->ConfigForwardSoftLimitEnable(true, 10);
    m_elevatorMotorA->ConfigForwardSoftLimitThreshold(
        ELEVATOR_HEIGHT_SOFT_LIMIT / ELEVATOR_INCHES_PER_CLICK, 10);
    m_elevatorMotorA->ConfigReverseSoftLimitEnable(true, 10);
    m_elevatorMotorA->ConfigReverseSoftLimitThreshold(
        ELEVATOR_HEIGHT_SOFT_LIMIT / ELEVATOR_INCHES_PER_CLICK, 10);

    m_elevatorMotorB->Follow(*m_elevatorMotorA);
    m_elevatorMotorB->SetInverted(true);

    m_elevatorMotorA->Set(ControlMode::PercentOutput, 0.0);

    m_positionCell = new LogCell("Elevator Position", 32, true);
    logger->RegisterCell(m_positionCell);
}

Elevator::~Elevator() {
    m_scheduler->UnregisterTask(this);
}

void Elevator::SetManualInput() {
    m_elevatorState = ElevatorState::joystickControl;
}

void Elevator::SetPower(double power) {
    m_elevatorState = ElevatorState::manualVoltage;
    m_elevatorMotorA->Set(ControlMode::PercentOutput, power);
}

void Elevator::SetPosition(double position) {
    m_elevatorState = ElevatorState::motionMagic;
    int position_clicks = position / ELEVATOR_INCHES_PER_CLICK;
    m_elevatorMotorA->Set(ControlMode::MotionMagic, position_clicks);
}

float Elevator::GetPosition() const {
    return ELEVATOR_INCHES_PER_CLICK *
           ((float)m_elevatorMotorA->GetSelectedSensorPosition(0));
}

void Elevator::ZeroPosition() {
    m_elevatorMotorA->GetSensorCollection().SetQuadraturePosition(
        ELEVATOR_HALL_HEIGHT_OFFSET / ELEVATOR_INCHES_PER_CLICK, 0);
}

void Elevator::EnableBrakeMode() {
    m_elevatorMotorA->SetNeutralMode(NeutralMode::Brake);
}

void Elevator::EnableCoastMode() {
    m_elevatorMotorA->SetNeutralMode(NeutralMode::Coast);
}

bool Elevator::GetElevatorHall() {
    return !m_elevatorHall->Get();
}

void Elevator::HallZero() {
    bool hallState = GetElevatorHall();
    if (m_prevHall != hallState) {
        ZeroPosition();
        m_prevHall = hallState;
        DBStringPrintf(DBStringPos::DB_LINE7, "1");
    }
    else {
        DBStringPrintf(DBStringPos::DB_LINE7, "0");
    }
}

void Elevator::TaskPeriodic(RobotMode mode) {
    m_positionCell->LogDouble(GetPosition());
    SmartDashboard::PutNumber("elevator/encoders/encoder", GetPosition());
    SmartDashboard::PutNumber("elevator/outputs/current",
                              m_elevatorMotorA->GetOutputCurrent());
    SmartDashboard::PutNumber("elevator/motorA/velocity",
                              m_elevatorMotorA->GetSelectedSensorVelocity(0));
    SmartDashboard::PutNumber("elevator/motorB/velocity",
                              m_elevatorMotorB->GetSelectedSensorVelocity(0));
    DBStringPrintf(DBStringPos::DB_LINE0, "e: %2.2lf", GetPosition());

    HallZero();

    switch (m_elevatorState) {
        case joystickControl:
            m_joystickControl =
                -m_operatorJoystick->GetRawAxisWithDeadband(Xbox::RightYAxis);
            if (GetElevatorHall()) {
                m_elevatorMotorA->Set(
                    ControlMode::PercentOutput,
                    pow(Util::bound(m_joystickControl, 0.0, 1.0), 3.0) / 3.0);
            }
            else if (GetPosition() < 15.0) {
                m_elevatorMotorA->Set(
                    ControlMode::PercentOutput,
                    pow(Util::bound(m_joystickControl, -0.67, 1.0), 3.0) / 3.0 +
                        ELEVATOR_FEED_FORWARD);
            }
            else {
                m_elevatorMotorA->Set(
                    ControlMode::PercentOutput,
                    pow(m_joystickControl, 3.0) / 3.0 + ELEVATOR_FEED_FORWARD);
            }
            break;
        case manualVoltage:
            break;
        case motionMagic:
            break;
        case idle:
            break;
        default:
            break;
    }
}
}
