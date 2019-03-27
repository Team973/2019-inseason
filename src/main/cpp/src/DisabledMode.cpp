#include "src/DisabledMode.h"
#include "src/Robot.h"

using namespace frc;

namespace frc973 {
Disabled::Disabled(ObservablePoofsJoystick *driver,
                   ObservableXboxJoystick *codriver, Elevator *elevator,
                   CargoIntake *cargoIntake, Limelight *limelightHatch,
                   Autonomous *autonomous)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_elevator(elevator)
        , m_cargoIntake(cargoIntake)
        , m_limelightHatch(limelightHatch)
        , m_autonomous(autonomous) {
}

Disabled::~Disabled() {
}
void Disabled::DisabledInit() {
    std::cout << "Disabled Start" << std::endl;
    m_elevator->EnableBrakeMode();
    m_cargoIntake->EnableBrakeMode();
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
