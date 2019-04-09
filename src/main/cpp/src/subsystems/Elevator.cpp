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
        , m_power(0.0)
        , m_joystickControl(0.0)
        , m_prevHall(true)
        , m_zeroingTime(0)
        , m_elevatorState(ElevatorState::idle)
        , m_rocketScoreMode(RocketScoreMode::low) {
    this->m_scheduler->RegisterTask("Elevator", this, TASK_PERIODIC);

    m_elevatorMotorA->ConfigSelectedFeedbackSensor(
        ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0,
        10);  // 0 = Not cascaded PID Loop; 10 = in constructor, not in a loop
    m_elevatorMotorA->SetSensorPhase(false);
    m_elevatorMotorA->SetNeutralMode(NeutralMode::Coast);
    m_elevatorMotorA->SetInverted(false);

    m_elevatorMotorA->Config_PID(0, 1.5, 0.0, 0.0, 0.0, 10);
    m_elevatorMotorA->ConfigMotionCruiseVelocity(3750.0 * 2.0, 10);
    m_elevatorMotorA->ConfigMotionAcceleration(5000.0 * 2.0, 10);
    m_elevatorMotorA->SelectProfileSlot(0, 0);

    m_elevatorMotorA->EnableCurrentLimit(true);
    m_elevatorMotorA->ConfigPeakCurrentDuration(0, 10);
    m_elevatorMotorA->ConfigPeakCurrentLimit(0, 10);
    m_elevatorMotorA->ConfigContinuousCurrentLimit(35, 10);
    m_elevatorMotorA->EnableVoltageCompensation(false);
    m_elevatorMotorA->ConfigForwardSoftLimitEnable(true, 10);
    m_elevatorMotorA->ConfigForwardSoftLimitThreshold(
        ELEVATOR_HEIGHT_SOFT_LIMIT / ELEVATOR_INCHES_PER_CLICK, 10);

    m_elevatorMotorB->Follow(*m_elevatorMotorA);
    m_elevatorMotorB->SetInverted(true);

    m_elevatorMotorA->Set(ControlMode::PercentOutput, 0.0);

    m_positionCell = new LogCell("Elevator Position", 32, true);
    m_currentMasterCell = new LogCell("Elevator Master Current", 32, true);
    m_currentFollowerCell = new LogCell("Elevator Follower Current", 32, true);
    m_voltageMasterCell = new LogCell("Elevator Master Voltage", 32, true);
    m_voltageFollowerCell = new LogCell("Elevator Follower Voltage", 32, true);
    m_controlModeCell = new LogCell("ControlMode Elevator", 32, true);
    m_powerInputCell = new LogCell("Elevator Power Input", 32, true);

    logger->RegisterCell(m_positionCell);
    logger->RegisterCell(m_currentMasterCell);
    logger->RegisterCell(m_currentFollowerCell);
    logger->RegisterCell(m_voltageMasterCell);
    logger->RegisterCell(m_voltageFollowerCell);
    logger->RegisterCell(m_controlModeCell);
    logger->RegisterCell(m_powerInputCell);
}

Elevator::~Elevator() {
    m_scheduler->UnregisterTask(this);
}

void Elevator::SetManualInput() {
    m_elevatorState = ElevatorState::joystickControl;
}

void Elevator::SetPower(double power) {
    m_elevatorState = ElevatorState::manualVoltage;
    m_power = power;
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

void Elevator::SetSoftLimit(double limit) {
    m_elevatorMotorA->ConfigForwardSoftLimitThreshold(
        limit / ELEVATOR_INCHES_PER_CLICK, 10);
}

void Elevator::HallZero() {
    bool hallState = GetElevatorHall();
    if (m_prevHall != hallState) {
        ZeroPosition();
        m_prevHall = hallState;
    }
}

void Elevator::SetRocketScoreMode(Elevator::RocketScoreMode mode) {
    m_rocketScoreMode = mode;
}

Elevator::RocketScoreMode Elevator::GetRocketScoreMode() {
    return m_rocketScoreMode;
}
void Elevator::TaskPeriodic(RobotMode mode) {
    m_positionCell->LogDouble(GetPosition());
    m_currentMasterCell->LogDouble(m_elevatorMotorA->GetOutputCurrent());
    m_voltageMasterCell->LogDouble(m_elevatorMotorA->GetMotorOutputVoltage());
    m_controlModeCell->LogInt(m_elevatorState);
    m_powerInputCell->LogDouble(m_power);
    DBStringPrintf(DBStringPos::DB_LINE0, "ep:%2.2lf ev:%2.2lf", GetPosition(),
                   (float)m_elevatorMotorA->GetSelectedSensorVelocity(0));
    DBStringPrintf(DBStringPos::DB_LINE5, "eout:%2.2lf ec:%2.2lf",
                   m_elevatorMotorA->GetMotorOutputPercent(),
                   m_elevatorMotorA->GetOutputCurrent());
    HallZero();

    switch (m_elevatorState) {
        case joystickControl:
            m_joystickControl =
                -m_operatorJoystick->GetRawAxisWithDeadband(Xbox::RightYAxis);

            if (GetElevatorHall()) {
                m_elevatorMotorA->Set(
                    ControlMode::PercentOutput,
                    Util::bound(pow(m_joystickControl, 3.0), 0.0, 0.3));
            }
            else {
                m_elevatorMotorA->Set(ControlMode::PercentOutput,
                                      Util::bound(pow(m_joystickControl, 3.0) +
                                                      ELEVATOR_FEED_FORWARD,
                                                  -0.3, 0.3));
            }
            break;
        case manualVoltage:
            if (GetElevatorHall()) {
                m_elevatorMotorA->Set(ControlMode::PercentOutput,
                                      fmax(-0.17, m_power));
            }
            else {
                m_elevatorMotorA->Set(ControlMode::PercentOutput, m_power);
            }
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
