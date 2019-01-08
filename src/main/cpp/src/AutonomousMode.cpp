#include "src/AutonomousMode.h"
#include "src/DisabledMode.h"
#include "src/Robot.h"

using namespace frc;

namespace frc973 {
Autonomous::Autonomous(Disabled *disabled, Drive *drive, ADXRS450_Gyro *gyro,
                       GreyLight *greylight)
        : m_noAuto(new NoAuto())
        , m_forwardAuto(new ForwardAuto(drive))
        , m_disabled(disabled)
        , m_routine(m_noAuto)
        , m_drive(drive)
        , m_gyro(gyro)
        , m_greylight(greylight)
        , m_autoSignal(new LightPattern::AutoIndicator()) {
}

Autonomous::~Autonomous() {
}

void Autonomous::AutonomousInit() {
    // Remember to zero all sensors here
    m_gyro->Reset();
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
