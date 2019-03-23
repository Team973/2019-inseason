#include "src/subsystems/Stinger.h"

namespace frc973 {
Stinger::Stinger(TaskMgr *scheduler, LogSpreadsheet *logger,
                 GreyTalonSRX *stingerDriveMotor)
        : m_scheduler(scheduler)
        , m_logger(logger)
        , m_stingerDriveMotor(stingerDriveMotor)
        , m_kickOffPneumatic(
              new DoubleSolenoid(PCM_CAN_ID, KICKOFF_FORWARD, KICKOFF_REVERSE))
        , m_sneakyClimb(new DoubleSolenoid(PCM_CAN_ID, SNEAKY_CLIMB_FORWARD,
                                           SNEAKY_CLIMB_REVERSE))
        , m_gateLatch(new Solenoid(PCM_CAN_ID, STINGER_GATE_LATCH_PCM_ID)) {
    this->m_scheduler->RegisterTask("Stinger", this, TASK_PERIODIC);

    m_kickOffPneumatic->Set(DoubleSolenoid::Value::kReverse);
    m_sneakyClimb->Set(DoubleSolenoid::Value::kReverse);

    m_current = new LogCell("Stinger Current", 32, true);
    m_logger->RegisterCell(m_current);
}

Stinger::~Stinger() {
    m_scheduler->UnregisterTask(this);
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

void Stinger::EngageGateLatch() {
    m_gateLatch->Set(true);
}

void Stinger::RetractGateLatch() {
    m_gateLatch->Set(false);
}

void Stinger::TaskPeriodic(RobotMode mode) {
    DBStringPrintf(DBStringPos::DB_LINE6, "pldrivec:%2.2lf",
                   m_stingerDriveMotor->GetOutputCurrent());
}
}
