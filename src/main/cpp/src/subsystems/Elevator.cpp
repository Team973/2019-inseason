#include "src/subsystems/Elevator.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/util/WrapDash.h"

using namespace frc;

namespace frc973 {
Elevator::Elevator(TaskMgr *scheduler, LogSpreadsheet *logger,
                   GreyTalonSRX *elevatorMotorA, VictorSPX *elevatorMotorB,
                   ObservableXboxJoystick *operatorJoystick)
        : m_scheduler(scheduler)
        , m_elevatorMotorA(elevatorMotorA)
        , m_elevatorMotorB(elevatorMotorB)
        , m_operatorJoystick(operatorJoystick)
        , m_position(0.0)
        , m_zeroingTime(0)
        , m_elevatorState(ElevatorState::manualVoltage) {
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
    m_elevatorState = ElevatorState::manualVoltage;
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
    m_elevatorMotorA->GetSensorCollection().SetQuadraturePosition(0, 0);
}

void Elevator::EnableBrakeMode() {
    m_elevatorMotorA->SetNeutralMode(NeutralMode::Brake);
}

void Elevator::EnableCoastMode() {
    m_elevatorMotorA->SetNeutralMode(NeutralMode::Coast);
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

    switch (m_elevatorState) {
        case manualVoltage:
            if (GetPosition() < 15.0 &&
                -m_operatorJoystick->GetRawAxisWithDeadband(Xbox::RightYAxis) <
                    0.0) {
                m_elevatorMotorA->Set(ControlMode::PercentOutput, 0.0);
            }
            else {
                m_elevatorMotorA->Set(
                    ControlMode::PercentOutput,
                    pow(-m_operatorJoystick->GetRawAxisWithDeadband(
                            Xbox::RightYAxis),
                        3.0) /
                            3.0 +
                        ELEVATOR_FEED_FORWARD);
            }
            break;
        case motionMagic:
            break;
        default:
            break;
    }
}
}
