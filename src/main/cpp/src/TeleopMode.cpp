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
Teleop::Teleop(ObservablePoofsJoystick *driver,
               ObservableXboxJoystick *codriver,
               ObservableDualActionJoystick *testStick, Drive *drive,
               Elevator *elevator, HatchIntake *hatchIntake,
               CargoIntake *cargoIntake, Stinger *stinger,
               Limelight *limelightCargo, Limelight *limelightHatch,
               PresetHandlerDispatcher *presetDispatcher)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_testJoystick(testStick)
        , m_drive(drive)
        , m_driveMode(DriveMode::Cheesy)
        , m_elevator(elevator)
        , m_hatchIntake(hatchIntake)
        , m_cargoIntake(cargoIntake)
        , m_gameMode(GameMode::HatchInit)
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
    double y =  // m_operatorJoystick->GetRawAxisWithDeadband(Xbox::LeftYAxis);
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::LeftYAxis);
    double x =  // m_operatorJoystick->GetRawAxisWithDeadband(Xbox::RightXAxis);
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::RightXAxis);
    bool quickturn =
        m_driverJoystick->GetRawButton(PoofsJoysticks::RightBumper);
    bool softwareLowGear =
        m_driverJoystick->GetRawButton(PoofsJoysticks::RightTrigger);

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
        case DriveMode::RegularLimelightHatch:
            m_drive->RegularLimelightHatchDrive();
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
        case GameMode::CargoInit:
            m_cargoIntake->RetractPlatformWheel();
            m_wristResetTimer = GetMsecTime();
            m_hatchIntake->SetIdle();
            m_hatchIntake->ManualPuncherRetract();
            m_gameMode = GameMode::CargoPeriodic;
            break;
        case GameMode::CargoPeriodic:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: cargo");
            if (m_wristResetTimer - GetMsecTime() > 500) {
                m_cargoIntake->RetractWrist();
            }
            SmartDashboard::PutString("misc/limelight/currentLimelight",
                                      "cargo");
            break;
        case GameMode::HatchInit:
            m_cargoIntake->RetractPlatformWheel();
            m_cargoIntake->StopIntake();
            m_wristResetTimer = GetMsecTime();
            m_gameMode = GameMode::HatchPeriodic;
            break;
        case GameMode::HatchPeriodic:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: hatch");
            if (m_wristResetTimer - GetMsecTime() > 500) {
                m_cargoIntake->RetractWrist();
            }
            break;
        case GameMode::EndGameInit:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: endgameinit");
            m_limelightHatch->SetCameraDriver();
            m_limelightHatch->SetPiPSecondary();
            m_cargoIntake->StopIntake();
            m_hatchIntake->SetIdle();
            m_elevator->SetPosition(Elevator::PLATFORM);
            if (m_elevator->GetPosition() > Elevator::PLATFORM - 2.0) {
                m_cargoIntake->DeployPlatformWheel();
                m_cargoIntake->ExtendWrist();
                m_gameMode = GameMode::EndGamePeriodic;
            }
            break;
        case GameMode::EndGamePeriodic:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: endgameperiodic");
            m_drive->SetStingerOutput(y);
            m_driveMode = DriveMode::Cheesy;
            if (m_stinger->GetUpperHall() &&
                m_driverJoystick->GetRawButton(PoofsJoysticks::LeftTrigger)) {
                // m_limelightCargo->SetLightBlink();
            }
            break;
        case GameMode::RaiseIntake:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: raiseintake");
            m_elevator->SetPosition(10.0);
            m_gameMode = GameMode::ResetIntake;
            break;
        case GameMode::ResetIntake:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: resetintake");
            if (fabs(m_elevator->GetPosition() - 10.0) < 1.0) {
                m_cargoIntake->RetractWrist();
                m_cargoIntake->RetractPlatformWheel();
                m_gameMode = GameMode::EndGamePeriodic;
            }
            break;
    }

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

    if (fabs(m_operatorJoystick->GetRawAxisWithDeadband(Xbox::RightYAxis)) >
        0.2) {
        m_elevator->SetManualInput();
    }

    if (fabs(m_operatorJoystick->GetRawAxisWithDeadband(
            Xbox::LeftTriggerAxis)) > 0.25) {
        switch (m_gameMode) {
            case GameMode::CargoPeriodic:
                m_cargoIntake->RetractWrist();
                break;
            case GameMode::HatchPeriodic:
                m_hatchIntake->ManualPuncherRetract();
                break;
            case GameMode::EndGamePeriodic:
                // Task
                break;
        }
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
            case Xbox::Back:
                if (pressedP) {
                    m_gameMode = GameMode::EndGameInit;
                    m_elevator->SetSoftLimit(
                        Elevator::ENDGAME_HEIGHT_SOFT_LIMIT);
                    m_rumble = Rumble::on;
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case Xbox::DPadUpVirtBtn:  // Changes game mode to Endgame
                if (pressedP) {
                }
                break;
            case Xbox::DPadDownVirtBtn:
                break;
            case Xbox::DPadLeftVirtBtn:  // Changes game mode to Cargo
                if (pressedP) {
                    m_gameMode = GameMode::CargoInit;
                    m_elevator->SetSoftLimit(
                        Elevator::ELEVATOR_HEIGHT_SOFT_LIMIT);
                    m_rumble = Rumble::on;
                    m_limelightHatch->SetPiPSecondary();
                    m_limelightHatch->SetCameraDriver();
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case Xbox::DPadRightVirtBtn:  // Changes game mode to Hatch
                if (pressedP) {
                    m_gameMode = GameMode::HatchInit;
                    m_elevator->SetSoftLimit(
                        Elevator::ELEVATOR_HEIGHT_SOFT_LIMIT);
                    m_rumble = Rumble::on;
                    m_limelightHatch->SetPiPMain();
                    m_limelightHatch->SetCameraDriver();
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case Xbox::BtnY:  // Middle Elevator Preset
            case Xbox::BtnA:  // Low Preset
            case Xbox::BtnX:  // Cargo Bay Preset
                m_presetDispatcher->ElevatorDispatchPressedButtonToPreset(
                    this, button, pressedP);
                break;
            case Xbox::LeftBumper:   // Extend Intake
            case Xbox::RightBumper:  // Intake
            case Xbox::BtnB:
                m_presetDispatcher->IntakeBumperPresets(this, button, pressedP);
                break;
        }
    }
}

void Teleop::HandleDualActionJoystick(uint32_t port, uint32_t button,
                                      bool pressedP) {
    if (port == TEST_JOYSTICK_PORT) {
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
            case DualAction::LeftTrigger:
            case DualAction::RightBumper:
            case DualAction::RightTrigger:
                m_presetDispatcher->DriveDispatchJoystickButtons(this, button,
                                                                 pressedP);
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
}
