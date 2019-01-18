#include "src/subsystems/Elevator.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/util/WrapDash.h"

using namespace frc;

namespace frc973 {
Elevator::Elevator(TaskMgr *scheduler, LogSpreadsheet *logger,
                   GreyTalonSRX *elevatorMotor, Limelight *limelight)
        : m_scheduler(scheduler)
        , m_elevatorMotor(elevatorMotor)
        , m_position(0.0)
        , m_zeroingTime(0)
        , m_elevatorState(ElevatorState::manualVoltage)
        , m_limelightVerticalController(
              new LimelightVerticalController(limelight, elevatorMotor)) {
    this->m_scheduler->RegisterTask("Elevator", this, TASK_PERIODIC);

    m_elevatorMotor->ConfigSelectedFeedbackSensor(
        ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0,
        10);  // 0 = Not cascaded PID Loop; 10 = in constructor, not in a loop
    m_elevatorMotor->SetSensorPhase(true);
    m_elevatorMotor->SetNeutralMode(NeutralMode::Coast);
    m_elevatorMotor->SetInverted(true);

    m_elevatorMotor->Config_kP(0, 1.5, 10);
    m_elevatorMotor->Config_kI(0, 0.0, 10);
    m_elevatorMotor->Config_kD(0, 0.0, 10);
    m_elevatorMotor->Config_kF(0, 0.0, 10);
    m_elevatorMotor->ConfigMotionCruiseVelocity(3750.0, 10);
    m_elevatorMotor->ConfigMotionAcceleration(4200.0, 10);
    m_elevatorMotor->SelectProfileSlot(0, 0);

    m_elevatorMotor->EnableCurrentLimit(false);
    m_elevatorMotor->ConfigPeakCurrentDuration(0, 10);
    m_elevatorMotor->ConfigPeakCurrentLimit(0, 10);
    m_elevatorMotor->ConfigContinuousCurrentLimit(25, 10);
    m_elevatorMotor->EnableVoltageCompensation(false);
    m_elevatorMotor->ConfigForwardSoftLimitThreshold(
        ELEVATOR_SOFT_HEIGHT_LIMIT / ELEVATOR_INCHES_PER_CLICK, 10);
    m_elevatorMotor->ConfigForwardSoftLimitEnable(true, 10);

    m_elevatorMotor->Set(ControlMode::PercentOutput, 0.0);

    m_positionCell = new LogCell("Elevator Position", 32, true);
    logger->RegisterCell(m_positionCell);
}

Elevator::~Elevator() {
    m_scheduler->UnregisterTask(this);
}

void Elevator::SetPower(double power) {
    m_elevatorState = ElevatorState::manualVoltage;
    power = Util::bound(power, -0.6, 1.0);
    m_elevatorMotor->Set(ControlMode::PercentOutput, power);
}

void Elevator::SetPosition(double position) {
    m_elevatorState = ElevatorState::motionMagic;
    int position_clicks = position / ELEVATOR_INCHES_PER_CLICK;
    m_elevatorMotor->Set(ControlMode::MotionMagic, position_clicks);
}

void Elevator::EnableLimelightControl() {
    m_elevatorState = ElevatorState::limelight;
}

float Elevator::GetPosition() const {
    return ELEVATOR_INCHES_PER_CLICK *
           ((float)m_elevatorMotor->GetSelectedSensorPosition(0));
}

void Elevator::ZeroPosition() {
    m_elevatorMotor->GetSensorCollection().SetQuadraturePosition(0, 0);
}

void Elevator::EnableBrakeMode() {
    m_elevatorMotor->SetNeutralMode(NeutralMode::Brake);
}

void Elevator::EnableCoastMode() {
    m_elevatorMotor->SetNeutralMode(NeutralMode::Coast);
}

void Elevator::TaskPeriodic(RobotMode mode) {
    m_positionCell->LogDouble(GetPosition());
    SmartDashboard::PutNumber("elevator/encoders/encoder", GetPosition());
    SmartDashboard::PutNumber("elevator/outputs/current",
                              m_elevatorMotor->GetOutputCurrent());
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
