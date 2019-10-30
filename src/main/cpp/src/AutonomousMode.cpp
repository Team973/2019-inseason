#include "src/AutonomousMode.h"

namespace frc973 {
Autonomous::Autonomous(ObservablePoofsJoystick *driverJoystick,
                       ObservableXboxJoystick *operatorJoystick,
                       ObservableDualActionJoystick *tuningJoystick,
                       Teleop *teleop, PigeonIMU *gyro, Drive *drive,
                       CargoIntake *cargoIntake, HatchIntake *hatchIntake,
                       Elevator *elevator, Limelight *limelightHatch)
        : m_driverJoystick(driverJoystick)
        , m_operatorJoystick(operatorJoystick)
        , m_tuningJoystick(tuningJoystick)
        , m_teleop(teleop)
        , m_autoState(AutoState::NoAuto)
        , m_autoStateStartPosition(AutoStateStartPosition::LeftHabLevel2)
        , m_autoTimer(0.0)
        , m_autoStep(0)
        , m_gyro(gyro)
        , m_drive(drive)
        , m_cargoIntake(cargoIntake)
        , m_hatchIntake(hatchIntake)
        , m_elevator(elevator)
        , m_limelightHatch(limelightHatch) {
}

Autonomous::~Autonomous() {
}

void Autonomous::AutonomousInit() {
    // Remember to zero all sensors here
    m_teleop->TeleopInit();
    m_drive->Zero();

    if (m_driverJoystick->GetRawAxisWithDeadband(PoofsJoystick::RightXAxis) <
        -0.5) {
        m_direction = -1.0;
        m_autoStateStartPosition = AutoStateStartPosition::LeftHabLevel2;
    }
    else if (m_driverJoystick->GetRawAxisWithDeadband(
                 PoofsJoystick::RightXAxis) > 0.5) {
        m_autoStateStartPosition = AutoStateStartPosition::RightHabLevel2;
    }
    else {
        m_autoStateStartPosition = AutoStateStartPosition::CenterHab;
    }

    PIDDriveController *ctrl = m_drive->GetPIDDriveController();

    ctrl->GetDrivePID()->SetGains(0.025, 0.0, 0.00);
    ctrl->GetTurnPID()->SetGains(0.0205, 0.0, 0.00135);
    std::cout << "Autonomous Start" << std::endl;
}

void Autonomous::AutonomousPeriodic() {
    // Driver throttle stick movement beyond a "DEAD_BAND" 0.5 will put us in
    // manual and stay in manual
    if (m_autoState != AutoState::Manual &&
        fabs(m_driverJoystick->GetRawAxisWithDeadband(
            PoofsJoystick::LeftYAxis)) > 0.50) {
        m_autoState = AutoState::Manual;
    }

    switch (m_autoState) {
        case AutoState::ForwardAuto:
            ForwardAuto();
            break;
        case AutoState::SingleHatchAuto:
            SingleHatchAuto();
            break;
        case AutoState::TwoRocket:
            TwoRocketAuto();
            break;
        case AutoState::TwoCargoShip:
            TwoCargoShipAuto();
            break;
        case AutoState::CargoShipThenRocket:
            CargoShipThenRocketAuto();
            break;
        case AutoState::CargoToHuman:
            StartPlatformToCargo();
            CargoToHumanPlayer();
            break;
        case AutoState::Manual:
            m_teleop->TeleopPeriodic();
            break;
        case AutoState::NoAuto:
            NoAuto();
            break;
        default:
            m_teleop->TeleopPeriodic();
            break;
    }
}
void Autonomous::HandlePoofsJoystick(uint32_t port, uint32_t button,
                                     bool pressedP) {
    if (m_autoState == AutoState::Manual) {
        m_teleop->HandlePoofsJoystick(port, button, pressedP);
    }
}

void Autonomous::HandleXboxJoystick(uint32_t port, uint32_t button,
                                    bool pressedP) {
    if (m_autoState == AutoState::Manual) {
        m_teleop->HandleXboxJoystick(port, button, pressedP);
    }
}

void Autonomous::HandleDualActionJoystick(uint32_t port, uint32_t button,
                                          bool pressedP) {
    if (m_autoState == AutoState::Manual) {
        m_teleop->HandleDualActionJoystick(port, button, pressedP);
    }
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
