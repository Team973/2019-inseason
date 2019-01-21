#include "src/TestMode.h"

using namespace frc;

namespace frc973 {
Test::Test(ObservablePoofsJoystick *driver, ObservableXboxJoystick *codriver,
           Drive *drive)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_drive(drive) {
}

Test::~Test() {
}

void Test::TestInit() {
    std::cout << "Test Start" << std::endl;
    m_driveMode = DriveMode::Openloop;
}

void Test::TestPeriodic() {
    /**
     * Driver Joystick
     */
    double y = -m_driverJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis);
    double x =
        -m_driverJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis);
    bool softwareLowGear =
        m_driverJoystick->GetRawButton(DualAction::RightTrigger);

    if (m_driveMode == DriveMode::Openloop) {
        if (softwareLowGear) {
            x /= 3.0;
            y /= 3.0;
        }
        m_drive->OpenloopArcadeDrive(y, x);
    }

    /**
     * Operator Joystick
     */
}

void Test::TestStop() {
}

void Test::HandlePoofsJoystick(uint32_t port, uint32_t button, bool pressedP) {
    if (port == DRIVER_JOYSTICK_PORT) {
        switch (button) {
            case PoofsJoysticks::LeftTrigger:
                if (pressedP) {
                }
                else {
                }
                break;
            case PoofsJoysticks::RightTrigger:
                if (pressedP) {
                }
                else {
                }
                break;
            case PoofsJoysticks::LeftBumper:
                if (pressedP) {
                }
                else {
                }
                break;
            case PoofsJoysticks::RightBumper:
                if (pressedP) {
                }
                else {
                }
                break;
        }
    }
}

void Test::HandleXboxJoystick(uint32_t port, uint32_t button, bool pressedP) {
    if (port == OPERATOR_JOYSTICK_PORT) {
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
}

void Test::HandleDualActionJoystick(uint32_t port, uint32_t button,
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
