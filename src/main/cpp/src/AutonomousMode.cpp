#include "src/AutonomousMode.h"
#include "src/DisabledMode.h"
#include "src/Robot.h"

using namespace frc;

namespace frc973 {
Autonomous::Autonomous(ObservablePoofsJoystick *driver,
                       ObservableXboxJoystick *codriver,
                       ObservableDualActionJoystick *testJoystick,
                       Teleop *teleop, ADXRS450_Gyro *gyro, Drive *drive,
                       CargoIntake *cargoIntake, HatchIntake *hatchIntake,
                       Elevator *elevator)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_testJoystick(testJoystick)
        , m_teleop(teleop)
        , m_autoState(AutoState::Manual)
        , m_autoStateStartPosition(AutoStateStartPosition::LeftHabLevel2)
        , m_autoTimer(0.0)
        , m_direction(1.0)  // counterclockwise is positive
        , m_autoStep(0)
        , m_gyro(gyro)
        , m_drive(drive)
        , m_cargoIntake(cargoIntake)
        , m_hatchIntake(hatchIntake)
        , m_elevator(elevator) {
}

Autonomous::~Autonomous() {
}

void Autonomous::AutonomousInit() {
    // Remember to zero all sensors here
    m_teleop->TeleopInit();
    m_gyro->Reset();
    m_direction = 1.0;

    if (m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::RightXAxis) <
        0.0) {
        m_direction = -1.0;
        m_autoStateStartPosition = AutoStateStartPosition::LeftHabLevel2;
    }
    else if (m_driverJoystick->GetRawAxisWithDeadband(
                 PoofsJoysticks::RightXAxis) > 0.0) {
        m_autoStateStartPosition = AutoStateStartPosition::RightHabLevel2;
    }
    else {
        m_autoStateStartPosition = AutoStateStartPosition::CenterHab;
    }

    std::cout << "Autonomous Start" << std::endl;
}

void Autonomous::AutonomousPeriodic() {
    // Driver throttle stick movement beyond a "DEAD_BAND" 0.5 will put us in
    // manual and stay in manual
    if (m_autoState != AutoState::Manual &&
        fabs(m_driverJoystick->GetRawAxisWithDeadband(
            PoofsJoysticks::LeftYAxis)) > 0.50) {
        m_autoState = AutoState::Manual;
    }

    switch (m_autoState) {
        case AutoState::TwoRocket:
            TwoRocketAuto();
            break;
        case AutoState::TwoCargoShip:
            TwoCargoShipAuto();
            break;
        case AutoState::CargoShipThenRocket:
            CargoShipThenRocketAuto();
            break;
        case AutoState::Manual:
            m_teleop->TeleopPeriodic();
            break;
        default:
            m_teleop->TeleopPeriodic();
            break;
    }
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

Autonomous::AutoState Autonomous::GetAutoState() const {
    return m_autoState;
}

void Autonomous::SetAutoState(AutoState autoState) {
    m_autoState = autoState;
}
}
