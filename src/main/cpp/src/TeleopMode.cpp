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
               HatchIntake *hatchIntake, Limelight *limelightCargo,
               Limelight *limelightHatch)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_drive(drive)
        , m_driveMode(DriveMode::Cheesy)
        , m_hatchIntake(hatchIntake)
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
    }

    /**
     * Driver Joystick
     */
    double y = -m_operatorJoystick->GetRawAxisWithDeadband(Xbox::LeftYAxis);
    //-m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::LeftYAxis);
    double x = -m_operatorJoystick->GetRawAxisWithDeadband(Xbox::RightXAxis);
    //-m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::RightXAxis);
    bool quickturn =
        m_driverJoystick->GetRawButton(PoofsJoysticks::RightBumper);

    bool softwareLowGear =
        m_driverJoystick->GetRawButton(PoofsJoysticks::RightTrigger);

    switch (m_driveMode) {
        case DriveMode::Cheesy:
            if (softwareLowGear) {
                m_drive->CheesyDrive(y / 3.0, x / 3.0, quickturn, false);
            }
            else {
                m_drive->CheesyDrive(y, x, quickturn, false);
            }
        case DriveMode::Openloop:
            if (softwareLowGear) {
                m_drive->OpenloopArcadeDrive(y / 3.0, x / 3.0);
            }
            else {
                m_drive->OpenloopArcadeDrive(y, x);
            }
            break;
        case DriveMode::LimelightCargo:
            m_drive->LimelightCargoDrive();
            break;
        case DriveMode::LimelightHatch:
            m_drive->LimelightHatchDrive();
            break;
        case DriveMode::AssistedCheesy:
            m_drive->AssistedCheesyDrive(y, x, quickturn, false);
            break;
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
                    m_limelightHatch->SetPipelineIndex(2);
                    m_driveMode = DriveMode::LimelightHatch;
                }
                else {
                    m_driveMode = DriveMode::Cheesy;
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
                    // m_limelightCargoTimer = GetMsecTime();
                    // m_driveMode = DriveMode::LimelightCargo;
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
                    m_driveMode = DriveMode::AssistedCheesy;
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
                m_limelightHatch->SetPipelineIndex(2);
                m_driveMode = DriveMode::LimelightHatch;
            }
            else {
                m_driveMode = DriveMode::Cheesy;
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
                m_driveMode = DriveMode::LimelightCargo;
            }
            else {
                m_driveMode = DriveMode::Cheesy;
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
                m_driveMode = DriveMode::AssistedCheesy;
            }
            else {
                m_driveMode = DriveMode::Cheesy;
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
