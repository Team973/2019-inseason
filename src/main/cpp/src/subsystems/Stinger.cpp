#include "src/subsystems/Stinger.h"

namespace frc973 {
Stinger::Stinger(TaskMgr *scheduler, LogSpreadsheet *logger,
                 GreyTalonSRX *stingerElevatorMotor,
                 DigitalInput *stingerLowerHall, DigitalInput *stingerUpperHall)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_stingerElevatorMotor(stingerElevatorMotor)
        , m_stingerLowerHall(stingerLowerHall)
        , m_stingerUpperHall(stingerUpperHall) {
    this->m_scheduler->RegisterTask("Stinger", this, TASK_PERIODIC);
    m_stingerElevatorMotor->Set(ControlMode::PercentOutput, 0.0);
    m_stingerElevatorMotor->SetNeutralMode(NeutralMode::Coast);
    m_stingerElevatorMotor->EnableCurrentLimit(false);
    m_stingerElevatorMotor->ConfigSelectedFeedbackSensor(
        ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0,
        10);  // 0 = Not cascaded PID Loop; 10 = in constructor, not in a loop

    m_current = new LogCell("Stinger Current", 32, true);
    m_logger->RegisterCell(m_current);
}

Stinger::~Stinger() {
    m_scheduler->UnregisterTask(this);
}

void Stinger::StingerStart() {
}

void Stinger::StingerStop() {
}

bool Stinger::GetLowerHall() {
    return m_stingerLowerHall->Get();
}

bool Stinger::GetUpperHall() {
    return m_stingerUpperHall->Get();
}

Stinger::StingerElevatorStatus Stinger::GetStingerElevatorState() {
    if (!Stinger::GetLowerHall() && !Stinger::GetUpperHall()) {
        return StingerElevatorStatus::middle;
    }
    if (Stinger::GetLowerHall() && !Stinger::GetUpperHall()) {
        return StingerElevatorStatus::bottom;
    }
    if (!Stinger::GetLowerHall() && Stinger::GetUpperHall()) {
        return StingerElevatorStatus::top;
    }
    if (Stinger::GetLowerHall() && Stinger::GetUpperHall()) {
        return StingerElevatorStatus::error;
    }
}

void Stinger::SetStingerElevatorOutput(double input) {
    m_stingerElevatorMotor->Set(ControlMode::PercentOutput, input);
}

void Stinger::TaskPeriodic(RobotMode mode) {
    m_current->LogDouble(m_stingerElevatorMotor->GetOutputCurrent());
}
}
