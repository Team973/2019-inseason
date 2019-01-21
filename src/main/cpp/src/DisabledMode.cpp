#include "src/DisabledMode.h"
#include "src/Robot.h"

using namespace frc;

namespace frc973 {
Disabled::Disabled(ObservablePoofsJoystick *driver,
                   ObservableXboxJoystick *codriver, Limelight *limelightCargo,
                   Limelight *limelightHatch)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_limelightCargo(limelightCargo)
        , m_limelightHatch(limelightHatch) {
}

Disabled::~Disabled() {
}

void Disabled::DisabledInit() {
    std::cout << "Disabled Start" << std::endl;

    m_limelightCargo->SetCameraDriver();
    m_limelightHatch->SetCameraVision();
}

void Disabled::DisabledPeriodic() {
    DBStringPrintf(DBStringPos::DB_LINE2, "tv %3.1lf th %3.1lf",
                   m_limelightHatch->GetVerticalLength(),
                   m_limelightHatch->GetHorizontalLength());
    DBStringPrintf(DBStringPos::DB_LINE1, "xd %3.2lf",
                   m_limelightHatch->GetHorizontalDistance());
    DBStringPrintf(DBStringPos::DB_LINE3, "HI: %f",
                   m_limelightHatch->FindTargetSkew());
    DBStringPrintf(DBStringPos::DB_LINE0, "Ratios: %f",
                   (m_limelightHatch->GetHorizontalLength() /
                    m_limelightHatch->GetVerticalLength()) *
                       (6.0 / 15.0));
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
            }
            else {
            }
            break;
        case Xbox::BtnX:
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::BtnB:
            if (pressedP) {
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
