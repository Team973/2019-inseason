/*
 * TeleopMode.cpp
 *
 *  Created on: January 7, 2019
 *      Authors: Kyle, Chris
 *
 */
#include "src/TeleopMode.h"
#include "src/PresetHandlerDispatcher.h"
#include <cmath>

using namespace frc;

namespace frc973 {
Teleop::Teleop(ObservablePoofsJoystick *driver,
               ObservableXboxJoystick *codriver, Drive *drive,
               Elevator *elevator, HatchIntake *hatchIntake,
               CargoIntake *cargoIntake, Stinger *stinger,
               Limelight *limelightCargo, Limelight *limelightHatch,
               PresetHandlerDispatcher *presetDispatcher)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_drive(drive)
        , m_driveMode(DriveMode::Cheesy)
        , m_elevator(elevator)
        , m_hatchIntake(hatchIntake)
        , m_cargoIntake(cargoIntake)
        , m_gameMode(GameMode::Hatch)
        , m_stinger(stinger)
        , m_limelightCargo(limelightCargo)
        , m_limelightHatch(limelightHatch)
        , m_presetDispatcher(presetDispatcher)
        , m_rumble(Rumble::off) {
}

Teleop::~Teleop() {
}

void Teleop::TeleopInit() {
    std::cout << "Teleop Start" << std::endl;
    m_elevator->EnableCoastMode();
}

void Teleop::TeleopPeriodic() {
    DBStringPrintf(DBStringPos::DB_LINE3, "X Off: %f",
                   m_limelightHatch->GetXOffset());
    /**
     * Driver Joystick
     */
    double y =
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::LeftYAxis);
    double x =
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::RightXAxis);
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
            break;
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
        case DriveMode::AssistedCheesyHatch:
            m_drive->AssistedCheesyHatchDrive(y, x, false, false);
            break;
        case DriveMode::AssistedCheesyCargo:
            m_drive->AssistedCheesyCargoDrive(y, x, false, false);
            break;
    }

    switch (m_gameMode) {
        case GameMode::Cargo:
            SmartDashboard::PutString("misc/limelight/currentLimelight",
                                      "hatch");  // cargo
            break;
        case GameMode::Hatch:
            SmartDashboard::PutString("misc/limelight/currentLimelight",
                                      "hatch");
            break;
        case GameMode::EndGame:
            m_limelightCargo->SetCameraOff();     // Driver
            m_limelightHatch->SetCameraDriver();  // Off
            m_limelightHatch->SetLightBlink();
            m_limelightCargo->SetLightBlink();
            SmartDashboard::PutString("misc/limelight/currentLimelight",
                                      "hatch");  // cargo
            break;
    }

    /**
     * Operator Joystick
     */
    if (fabs(m_operatorJoystick->GetRawAxisWithDeadband(Xbox::RightYAxis)) >
        0.2) {
        m_elevator->SetManualInput();
    }

    m_presetDispatcher->PresetPeriodic(this);

    switch (m_rumble) {
        case Rumble::on:
            m_rumbleTimer = GetMsecTime();
            m_operatorJoystick->SetRumble(GenericHID ::RumbleType::kRightRumble,
                                          1);
            m_operatorJoystick->SetRumble(GenericHID ::RumbleType::kLeftRumble,
                                          1);
            break;
        case Rumble::off:
            if ((GetMsecTime() - m_rumbleTimer) > 150) {
                m_operatorJoystick->SetRumble(
                    GenericHID ::RumbleType::kRightRumble, 0);
                m_operatorJoystick->SetRumble(
                    GenericHID ::RumbleType::kLeftRumble, 0);
            }
            break;
    }
}

void Teleop::TeleopStop() {
}

void Teleop::HandlePoofsJoystick(uint32_t port, uint32_t button,
                                 bool pressedP) {
    if (port == DRIVER_JOYSTICK_PORT) {
        switch (button) {
            case PoofsJoysticks::LeftTrigger:
            case PoofsJoysticks::RightTrigger:  // Score
                m_presetDispatcher->DriveDispatchJoystickTrigger(this, button,
                                                                 pressedP);
                break;
            case PoofsJoysticks::LeftBumper:
            case PoofsJoysticks::RightBumper:
                m_presetDispatcher->DriveDispatchJoystickBumper(this, button,
                                                                pressedP);
                break;
        }
    }
}

void Teleop::HandleXboxJoystick(uint32_t port, uint32_t button, bool pressedP) {
    if (port == OPERATOR_JOYSTICK_PORT) {
        switch (button) {
            case Xbox::BtnY:  // High Elevator Preset
            case Xbox::BtnA:  // Low Preset
            case Xbox::BtnX:  // Cargo Bay Preset
            case Xbox::BtnB:  // Middle Elevator Preset
                m_presetDispatcher->ElevatorDispatchPressedButtonToPreset(
                    this, button, pressedP);
                break;
            case Xbox::LeftBumper:   // Extend Intake
            case Xbox::RightBumper:  // Intake
                m_presetDispatcher->IntakeBumperPresets(this, button, pressedP);
                break;
            case Xbox::DPadUpVirtBtn:  // Changes game mode to Endgame
                if (pressedP) {
                    m_gameMode = GameMode::EndGame;
                    m_rumble = Rumble::on;
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case Xbox::DPadDownVirtBtn:
                break;
            case Xbox::DPadLeftVirtBtn:  // Changes game mode to Cargo
                if (pressedP) {
                    m_gameMode = GameMode::Cargo;
                    m_hatchIntake->SetIdle();
                    m_hatchIntake->ManualPuncherRetract();
                    m_rumble = Rumble::on;
                    m_limelightCargo->SetCameraDriver();
                    m_limelightCargo->SetLightOn();
                    m_limelightHatch->SetCameraOff();
                    m_limelightHatch->SetLightOff();
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case Xbox::DPadRightVirtBtn:  // Changes game mode to Hatch
                if (pressedP) {
                    m_gameMode = GameMode::Hatch;
                    m_cargoIntake->StopIntake();
                    m_cargoIntake->RetractWrist();
                    m_rumble = Rumble::on;
                    m_limelightCargo->SetCameraOff();
                    m_limelightCargo->SetLightOff();
                    m_limelightHatch->SetCameraDriver();
                    m_limelightHatch->SetLightOn();
                }
                else {
                    m_rumble = Rumble::off;
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
            break;
        case DualAction::BtnB:
            if (pressedP) {
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
