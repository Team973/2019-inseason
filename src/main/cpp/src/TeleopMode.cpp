/*
 * TeleopMode.cpp
 *
 *  Created on: January 7, 2019
 *      Authors: Kyle, Chris
 *
 */
#include "src/TeleopMode.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "src/GameMode.h"
#include "src/PresetHandlerDispatcher.h"
#include <cmath>

using namespace frc;
using namespace nt;

namespace frc973 {
Teleop::Teleop(ObservableJoystickBase *driver, ObservableXboxJoystick *codriver,
               Drive *drive, Elevator *elevator, HatchIntake *hatchIntake,
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
    m_cargoIntake->EnableCoastMode();
}

void Teleop::TeleopPeriodic() {
    /**
     * Driver Joystick
     */
    const JoystickBase::Joystick &stick =
        m_driverJoystick->GetPeriodicJoystick();

    double y = -m_driverJoystick->GetRawAxisWithDeadband(stick.LeftYAxis);
    double x = -m_driverJoystick->GetRawAxisWithDeadband(stick.RightXAxis);
    bool quickturn = m_driverJoystick->GetRawButton(stick.RightBumper);
    bool softwareLowGear = m_driverJoystick->GetRawButton(stick.RightTrigger);

    if (m_stinger->GetLowerHall() && m_gameMode == GameMode::EndGamePeriodic) {
        softwareLowGear = true;
    }

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
            m_drive->AssistedCheesyHatchDrive(y, x, quickturn, false);
            break;
        case DriveMode::AssistedCheesyCargo:
            m_drive->AssistedCheesyCargoDrive(y, x, quickturn, false);
            break;
        default:
            m_drive->CheesyDrive(y, x, quickturn, false);
            break;
    }

    switch (m_gameMode) {
        case GameMode::Cargo:
            m_limelightCargo->SetLightOn();
            m_limelightHatch->SetLightOff();
            m_cargoIntake->RetractPlatformWheel();
            SmartDashboard::PutString("misc/limelight/currentLimelight",
                                      "cargo");
            break;
        case GameMode::Hatch:
            m_limelightCargo->SetLightOff();
            m_limelightHatch->SetLightOn();
            m_cargoIntake->RetractPlatformWheel();
            break;
        case GameMode::EndGameInit:
            m_limelightCargo->SetLightBlink();
            m_limelightCargo->SetCameraDriver();
            m_limelightHatch->SetCameraOff();
            m_limelightHatch->SetLightBlink();
            m_elevator->SetPosition(Elevator::PLATFORM);
            if (m_elevator->GetPosition() > Elevator::PLATFORM - 1.0) {
                m_cargoIntake->DeployPlatformWheel();
                m_cargoIntake->ExtendWrist();
                m_gameMode = GameMode::EndGamePeriodic;
            }
            break;
        case GameMode::EndGamePeriodic:
            m_drive->SetStingerOutput(y);
            m_driveMode = DriveMode::Cheesy;
            break;
        case GameMode::RaiseIntake:
            m_elevator->SetPosition(10.0);
            m_gameMode = GameMode::ResetIntake;
            break;
        case GameMode::ResetIntake:
            if (fabs(m_elevator->GetPosition() - 10.0) < 1.0) {
                m_cargoIntake->RetractWrist();
                m_cargoIntake->RetractPlatformWheel();
                m_gameMode = GameMode::EndGamePeriodic;
            }
            break;
    }

    m_presetDispatcher->JoystickPeriodic(this);  // RightYAxis & LeftTriggerAxis

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
            case PoofsJoysticks::LeftBumper:
            case PoofsJoysticks::RightBumper:
                m_presetDispatcher->DriveDispatchJoystickButtons(this, button,
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
                m_presetDispatcher->ElevatorDispatchPressedButtonToPreset(
                    this, button, pressedP);
                break;
            case Xbox::LeftBumper:   // Extend Intake
            case Xbox::RightBumper:  // Intake
            case Xbox::BtnB:         // Middle Elevator Preset
                m_presetDispatcher->IntakeBumperPresets(this, button, pressedP);
                break;
            case Xbox::DPadUpVirtBtn:  // Changes game mode to Endgame
                if (pressedP) {
                    m_gameMode = GameMode::EndGameInit;
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
                    m_cargoIntake->RetractPlatformWheel();
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
    if (port == DRIVER_DA_JOYSTICK_PORT) {
        switch (button) {
            case DualAction::LeftBumper:
            case DualAction::LeftTrigger:
            case DualAction::RightBumper:
            case DualAction::RightTrigger:
                m_presetDispatcher->DriveDispatchJoystickButtons(this, button,
                                                                 pressedP);
                break;
        }
    }
    else if (port == OPERATOR_DA_JOYSTICK_PORT) {
        switch (button) {
            case DualAction::BtnY:
            case DualAction::BtnA:  // Low Preset
            case DualAction::BtnX:  // Cargo Bay Preset
                m_presetDispatcher->ElevatorDispatchPressedButtonToPreset(
                    this, button, pressedP);
                break;
            case DualAction::LeftBumper:   // Extend Intake
            case DualAction::RightBumper:  // Intake
            case DualAction::BtnB:
                m_presetDispatcher->IntakeBumperPresets(this, button, pressedP);
                break;
            case DualAction::DPadUpVirtBtn:  // Changes game mode to Endgame
                if (pressedP) {
                    m_gameMode = GameMode::EndGameInit;
                    m_rumble = Rumble::on;
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case DualAction::DPadDownVirtBtn:
                break;
            case DualAction::DPadLeftVirtBtn:  // Changes game mode to Cargo
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
            case DualAction::DPadRightVirtBtn:  // Changes game mode to Hatch
                if (pressedP) {
                    m_gameMode = GameMode::Hatch;
                    m_cargoIntake->StopIntake();
                    m_cargoIntake->RetractWrist();
                    m_cargoIntake->RetractPlatformWheel();
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
}
