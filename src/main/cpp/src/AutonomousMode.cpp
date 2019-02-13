#include "src/AutonomousMode.h"
#include "src/DisabledMode.h"
#include "src/Robot.h"
#include "src/PresetHandlerDispatcher.h"

using namespace frc;

namespace frc973 {
Autonomous::Autonomous(Disabled *disabled, Drive *drive, Elevator *elevator,
                       ADXRS450_Gyro *gyro,
                       PresetHandlerDispatcher *presetDispatcher)
        : m_noAuto(new NoAuto())
        , m_forwardAuto(new ForwardAuto(drive))
        , m_disabled(disabled)
        , m_routine(m_noAuto)
        , m_drive(drive)
        , m_elevator(elevator)
        , m_gyro(gyro)
        , m_presetDispatcher(presetDispatcher)
        , m_gameMode(GameMode::Hatch) {
}

Autonomous::~Autonomous() {
}

void Autonomous::AutonomousInit() {
    // Remember to zero all sensors here
    m_gyro->Reset();
    m_elevator->EnableCoastMode();
    std::cout << "Autonomous Start" << std::endl;

    m_forwardAuto->Reset();
    m_routine = m_forwardAuto;
}

void Autonomous::AutonomousPeriodic() {
    m_routine->Execute();

    // Match time to display in dashboard
    SmartDashboard::PutNumber("misc/timer",
                              DriverStation::GetInstance().GetMatchTime());
}

void Autonomous::AutonomousStop() {
}
}
