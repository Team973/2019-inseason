#include "src/AutonomousMode.h"
#include "src/DisabledMode.h"
#include "src/Robot.h"

using namespace frc;

namespace frc973 {
Autonomous::Autonomous(ObservablePoofsJoystick *driver,
                       ObservableXboxJoystick *codriver,
                       ObservableDualActionJoystick *testJoystick,
                       Teleop *teleop)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_testJoystick(testJoystick)
        , m_teleop(teleop) {
}

Autonomous::~Autonomous() {
}

void Autonomous::AutonomousInit() {
    // Remember to zero all sensors here
    m_teleop->TeleopInit();
    std::cout << "Autonomous Start" << std::endl;
}

void Autonomous::AutonomousPeriodic() {
    m_teleop->TeleopPeriodic();
}
void Autonomous::HandlePoofsJoystick(uint32_t port, uint32_t button,
                                     bool pressedP) {
    m_teleop->HandlePoofsJoystick(port, button, pressedP);
}

void Autonomous::HandleXboxJoystick(uint32_t port, uint32_t button,
                                    bool pressedP) {
    m_teleop->HandleXboxJoystick(port, button, pressedP);
}

void Autonomous::HandleDualActionJoystick(uint32_t port, uint32_t button,
                                          bool pressedP) {
    m_teleop->HandleDualActionJoystick(port, button, pressedP);
}
void Autonomous::AutonomousStop() {
    m_teleop->TeleopStop();
}
}
