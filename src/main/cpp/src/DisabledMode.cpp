#include "src/DisabledMode.h"

namespace frc973 {
Disabled::Disabled(ObservablePoofsJoystick *driverJoystick,
                   ObservableXboxJoystick *operatorJoystick, Elevator *elevator,
                   CargoIntake *cargoIntake, Drive *drive,
                   Limelight *limelightHatch, Autonomous *autonomous,
                   Teleop *teleop)
        : m_driverJoystick(driverJoystick)
        , m_operatorJoystick(operatorJoystick)
        , m_elevator(elevator)
        , m_cargoIntake(cargoIntake)
        , m_drive(drive)
        , m_limelightHatch(limelightHatch)
        , m_autonomous(autonomous)
        , m_teleop(teleop) {
}

Disabled::~Disabled() {
}
void Disabled::DisabledInit() {
    std::cout << "Disabled Start" << std::endl;
    m_elevator->EnableBrakeMode();
    m_cargoIntake->EnableBrakeMode();
    if (m_teleop->GetGameMode() ==
            Teleop::GameMode::ThirdLevelEndGamePeriodic ||
        m_teleop->GetGameMode() ==
            Teleop::GameMode::SecondLevelEndGamePeriodic) {
        m_drive->EnableBrakeMode();
    }
}

void Disabled::DisabledPeriodic() {
}

void Disabled::DisabledStop() {
}

void Disabled::HandleXboxJoystick(uint32_t port, uint32_t button,
                                  bool pressedP) {
    switch (button) {
        case Xbox::BtnY:
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::BtnA:
            if (pressedP) {
                m_autonomous->SetAutoState(Autonomous::AutoState::TwoCargoShip);
            }
            else {
            }
            break;
        case Xbox::BtnX:
            if (pressedP) {
                m_autonomous->SetAutoState(
                    Autonomous::AutoState::CargoShipThenRocket);
            }
            else {
            }
            break;
        case Xbox::BtnB:
            if (pressedP) {
                m_autonomous->SetAutoState(Autonomous::AutoState::TwoRocket);
            }
            else {
            }
            break;
        case Xbox::LeftBumper:
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::LJoystickBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::RJoystickBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::RightBumper:
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::DPadUpVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::DPadDownVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::DPadLeftVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::DPadRightVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::Back:
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::Start:
            if (pressedP) {
            }
            else {
            }
            break;
    }
}

void Disabled::HandleDualActionJoystick(uint32_t port, uint32_t button,
                                        bool pressedP) {
    switch (button) {
        case DualAction::BtnA:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::BtnY:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::LeftBumper:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::LeftTrigger:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::RightBumper:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::RightTrigger:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::DPadUpVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::DPadDownVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::DPadLeftVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::DPadRightVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        default:
            break;
    }
}
}
