#include "src/AutonomousMode.h"
#include "src/DisabledMode.h"
#include "src/Robot.h"

using namespace frc;

namespace frc973 {
Autonomous::Autonomous(Disabled *disabled, Drive *drive, ADXRS450_Gyro *gyro,
                       ObservablePoofsJoystick *driver,
                       ObservableXboxJoystick *codriver)
        : m_disabled(disabled)
        , m_drive(drive)
        , m_gyro(gyro)
        , m_driveMode(DriveMode::Cheesy)
        , m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_noAuto(new NoAuto())
        , m_forwardAuto(new ForwardAuto(drive)) {
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
    /**
     * Driver Joystick
     */
    double y =
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::LeftYAxis);
    double x =
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::RightXAxis);
    bool quickturn =
        m_driverJoystick->GetRawButton(PoofsJoysticks::RightBumper);
    bool lowGear = m_driverJoystick->GetRawButton(PoofsJoysticks::RightTrigger);

    if (m_driveMode == DriveMode::Cheesy) {
        if (lowGear) {
            m_drive->CheesyDrive(y / 3.0, x / 3.0, quickturn, false);
        }
        else {
            m_drive->CheesyDrive(y, x, quickturn, false);
        }
    }
    else if (m_driveMode == DriveMode::Openloop) {
        if (lowGear) {
            m_drive->OpenloopArcadeDrive(y / 3.0, x / 3.0);
        }
        else {
            m_drive->OpenloopArcadeDrive(y, x);
        }
    }

    /**
     * Operator Joystick
     */
}
void Autonomous::HandlePoofsJoystick(uint32_t port, uint32_t button,
                                     bool pressedP) {
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

void Autonomous::HandleXboxJoystick(uint32_t port, uint32_t button,
                                    bool pressedP) {
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

void Autonomous::HandleDualActionJoystick(uint32_t port, uint32_t button,
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
void Autonomous::AutonomousStop() {
}
}
