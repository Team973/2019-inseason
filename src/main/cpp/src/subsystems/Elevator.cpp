#include "src/subsystems/Elevator.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/util/WrapDash.h"

using namespace frc;

namespace frc973 {
Elevator::Elevator(TaskMgr *scheduler, LogSpreadsheet *logger,
                   GreyTalonSRX *elevatorMotorA, VictorSPX *elevatorMotorB,
                   Limelight *limelight)
        : m_scheduler(scheduler)
        , m_elevatorMotorA(elevatorMotorA)
        , m_elevatorMotorB(elevatorMotorB)
        , m_position(0.0)
        , m_zeroingTime(0)
        , m_elevatorState(ElevatorState::manualVoltage)
        , m_limelightVerticalController(
              new LimelightVerticalController(limelight, elevatorMotorA)) {
    this->m_scheduler->RegisterTask("Elevator", this, TASK_PERIODIC);

    m_elevatorMotorA->ConfigSelectedFeedbackSensor(
        ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0,
        10);  // 0 = Not cascaded PID Loop; 10 = in constructor, not in a loop
    m_elevatorMotorA->SetSensorPhase(true);
    m_elevatorMotorA->SetNeutralMode(NeutralMode::Coast);
    m_elevatorMotorA->SetInverted(true);

    m_elevatorMotorA->Config_kP(0, 1.5, 10);
    m_elevatorMotorA->Config_kI(0, 0.0, 10);
    m_elevatorMotorA->Config_kD(0, 0.0, 10);
    m_elevatorMotorA->Config_kF(0, 0.0, 10);
    m_elevatorMotorA->ConfigMotionCruiseVelocity(3750.0, 10);
    m_elevatorMotorA->ConfigMotionAcceleration(4200.0, 10);
    m_elevatorMotorA->SelectProfileSlot(0, 0);

    m_elevatorMotorA->EnableCurrentLimit(true);
    m_elevatorMotorA->ConfigPeakCurrentDuration(0, 10);
    m_elevatorMotorA->ConfigPeakCurrentLimit(0, 10);
    m_elevatorMotorA->ConfigContinuousCurrentLimit(60, 10);
    m_elevatorMotorA->EnableVoltageCompensation(false);
    m_elevatorMotorA->ConfigForwardSoftLimitThreshold(
        ELEVATOR_SOFT_HEIGHT_LIMIT / ELEVATOR_INCHES_PER_CLICK, 10);
    m_elevatorMotorA->ConfigForwardSoftLimitEnable(true, 10);

    m_elevatorMotorB->Follow(*m_elevatorMotorA);
    m_elevatorMotorB->SetInverted(false);

    m_elevatorMotorA->Set(ControlMode::PercentOutput, 0.0);

    m_positionCell = new LogCell("Elevator Position", 32, true);
    logger->RegisterCell(m_positionCell);
}

Elevator::~Elevator() {
    m_scheduler->UnregisterTask(this);
}

void Elevator::SetPower(double power) {
    m_elevatorState = ElevatorState::manualVoltage;
    power = Util::bound(power, -0.6, 1.0);
    m_elevatorMotorA->Set(ControlMode::PercentOutput, power);
}

void Elevator::SetPosition(double position) {
    m_elevatorState = ElevatorState::motionMagic;
    int position_clicks = position / ELEVATOR_INCHES_PER_CLICK;
    m_elevatorMotorA->Set(ControlMode::MotionMagic, position_clicks);
}

void Elevator::EnableLimelightControl() {
    m_elevatorState = ElevatorState::limelight;
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
    switch (m_elevatorState) {
        case manualVoltage:
            break;
        case motionMagic:
            break;
        case limelight:
            m_limelightVerticalController->CalcOutput();
            break;
        default:
            break;
    }
}
}
