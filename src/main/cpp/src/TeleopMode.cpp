/*
 * TeleopMode.cpp
 *
 *  Created on: January 7, 2018
 *      Authors: Kyle, Chris
 *
 */
#include "src/TeleopMode.h"
#include <cmath>

using namespace frc;

namespace frc973 {
Teleop::Teleop(ObservablePoofsJoystick *driver,
               ObservableXboxJoystick *codriver, Drive *drive,
               GreyLight *greylight, Limelight *limelightCargo,
               Limelight *limelightHatch)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_drive(drive)
        , m_driveMode(DriveMode::Openloop)
        , m_greylight(greylight)
        , m_endGameSignal(
              new LightPattern::Flash(END_GAME_RED, NO_COLOR, 50, 15))
        , m_endGameSignalSent(false)
        , m_limelightCargo(limelightCargo)
        , m_limelightHatch(limelightHatch) {
}

Teleop::~Teleop() {
}

void Teleop::TeleopInit() {
    std::cout << "Teleop Start" << std::endl;
}

void Teleop::TeleopPeriodic() {
    if (!m_endGameSignalSent && Timer::GetMatchTime() < 40) {
        m_endGameSignalSent = true;
        m_endGameSignal->Reset();
        m_greylight->SetPixelStateProcessor(m_endGameSignal);
    }

    /**
     * Driver Joystick
     */
    double y =
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::LeftYAxis);
    double x =
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::RightXAxis);
    bool quickturn =
        m_driverJoystick->GetRawButton(PoofsJoysticks::RightBumper);

    if (m_driveMode == DriveMode::Openloop) {
        // if (softwareLowGear) {
        //     x /= 3.0;
        //     y /= 3.0;
        // }
        m_drive->OpenloopArcadeDrive(y, x);
    }
    else if (m_driveMode == DriveMode::Cheesy) {
        m_drive->CheesyDrive(y, x, quickturn, false);
    }
    else if (m_driveMode == DriveMode::LimelightCargo) {
        m_drive->LimelightCargoDrive(y, x);
    }
    else if (m_driveMode == DriveMode::LimelightHatch) {
        m_drive->LimelightHatchDrive(y, x);
    }

    /**
     * Operator Joystick
     */

    if (m_operatorJoystick->GetRawButton(Xbox::LeftBumper) &&
        m_limelightCargo->isTargetValid() == 1) {
        printf("got a Cargo target\n");
        printf("%d\n", (GetMsecTime() - m_limelightCargoTimer));
    }
    if (m_operatorJoystick->GetRawButton(Xbox::RightBumper) &&
        m_limelightHatch->isTargetValid() == 1) {
        printf("got a hatch target\n");
        printf("%d\n", (GetMsecTime() - m_limelightHatchTimer));
    }
}

void Teleop::TeleopStop() {
}

void Teleop::HandlePoofsJoystick(uint32_t port, uint32_t button,
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

void Teleop::HandleXboxJoystick(uint32_t port, uint32_t button, bool pressedP) {
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
                    m_limelightCargoTimer = GetMsecTime();
                    m_driveMode = DriveMode::LimelightCargo;
                }
                else {
                    m_driveMode = DriveMode::Cheesy;
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
                    m_limelightHatchTimer = GetMsecTime();
                    m_driveMode = DriveMode::LimelightHatch;
                }
                else {
                    m_driveMode = DriveMode::Cheesy;
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

void Teleop::HandleDualActionJoystick(uint32_t port, uint32_t button,
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
