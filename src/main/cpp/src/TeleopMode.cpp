/*
 * TeleopMode.cpp
 *
 *  Created on: January 7, 2019
 *      Authors: Kyle, Chris
 *
 */
#include "src/TeleopMode.h"
#include <cmath>

using namespace frc;

namespace frc973 {
Teleop::Teleop(ObservablePoofsJoystick *driver,
               ObservableXboxJoystick *codriver, Drive *drive,
               Elevator *elevator, HatchIntake *hatchIntake,
               CargoIntake *cargoIntake, Limelight *limelightCargo,
               Limelight *limelightHatch)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_drive(drive)
        , m_driveMode(DriveMode::Cheesy)
        , m_elevator(elevator)
        , m_hatchIntake(hatchIntake)
        , m_cargoIntake(cargoIntake)
        , m_gameMode(GameMode::Hatch)
        , m_limelightCargo(limelightCargo)
        , m_limelightHatch(limelightHatch)
        , m_rumble(Rumble::off) {
}

Teleop::~Teleop() {
}

void Teleop::TeleopInit() {
    std::cout << "Teleop Start" << std::endl;
    m_elevator->EnableCoastMode();
}

void Teleop::TeleopPeriodic() {
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
        case DriveMode::AssistedCheesy:
            m_drive->AssistedCheesyDrive(y, x, quickturn, false);
            break;
    }

    switch (m_gameMode) {
        case GameMode::Cargo:
            m_limelightCargo->SetLightOn();
            m_limelightHatch->SetLightOff();
            SmartDashboard::PutString("misc/limelight/currentLimelight",
                                      "cargo");
            break;
        case GameMode::Hatch:
            m_limelightCargo->SetLightOff();
            m_limelightHatch->SetLightOn();
            SmartDashboard::PutString("misc/limelight/currentLimelight",
                                      "hatch");
            break;
        case GameMode::EndGame:
            m_limelightHatch->SetLightBlink();
            m_limelightCargo->SetLightBlink();
            SmartDashboard::PutString("misc/limelight/currentLimelight",
                                      "cargo");
            break;
    }

    /**
     * Operator Joystick
     */
    if (fabs(m_operatorJoystick->GetRawAxisWithDeadband(Xbox::RightYAxis)) >
        0.2) {
        m_elevator->SetManualInput();
    }

    if (fabs(m_operatorJoystick->GetRawAxisWithDeadband(Xbox::LeftBumper)) >
        0.25) {
        switch (m_gameMode) {
            case GameMode::Cargo:
                m_cargoIntake->GoToWristState(
                    CargoIntake::CargoWristState::retracted);
                break;
            case GameMode::Hatch:
                m_hatchIntake->ManualPuncherRetract();
                break;
            case GameMode::EndGame:
                // Task
                break;
        }
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
}

void Teleop::TeleopStop() {
}

void Teleop::HandlePoofsJoystick(uint32_t port, uint32_t button,
                                 bool pressedP) {
    if (port == DRIVER_JOYSTICK_PORT) {
        switch (button) {
            case PoofsJoysticks::LeftTrigger:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::Cargo:  // Assisted Cheesy
                            m_driveMode = DriveMode::AssistedCheesy;
                            break;
                        case GameMode::Hatch:  // Assisted Cheesy
                            m_driveMode = DriveMode::AssistedCheesy;
                            break;
                        case GameMode::EndGame:  // Climb Down
                            // Task
                            break;
                    }
                }
                else {
                    switch (m_gameMode) {
                        case GameMode::Cargo:  // Assisted Cheesy
                            m_driveMode = DriveMode::Cheesy;
                            break;
                        case GameMode::Hatch:  // Assisted Cheesy
                            m_driveMode = DriveMode::Cheesy;
                            break;
                        case GameMode::EndGame:  // Climb Down
                            // Task
                            break;
                    }
                }
                break;
            case PoofsJoysticks::RightTrigger:  // Score
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::Cargo:  // Score Cargo
                            m_cargoIntake->Exhaust();
                            break;
                        case GameMode::Hatch:  // Score Hatch
                            m_hatchIntake->Exhaust();
                            break;
                        case GameMode::EndGame:  // Raise Intake
                            // Task
                            break;
                    }
                }
                else {
                    switch (m_gameMode) {
                        case GameMode::Cargo:
                            m_cargoIntake->StopIntake();
                            break;
                        case GameMode::Hatch:
                            m_hatchIntake->SetIdle();
                            break;
                        case GameMode::EndGame:
                            // Task
                            break;
                    }
                }
                break;
            case PoofsJoysticks::LeftBumper:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::Cargo:  // Auto Score Cargo
                            m_driveMode = DriveMode::LimelightCargo;
                            break;
                        case GameMode::Hatch:  // Auto Score Hatch
                            m_driveMode = DriveMode::LimelightHatch;
                            break;
                        case GameMode::EndGame:  // Climb Up Stinger
                            // Task
                            break;
                    }
                }
                else {
                    switch (m_gameMode) {
                        case GameMode::Cargo:
                            m_driveMode = DriveMode::Cheesy;
                            break;
                        case GameMode::Hatch:
                            m_driveMode = DriveMode::Cheesy;
                            break;
                        case GameMode::EndGame:
                            // Task
                            break;
                    }
                }
                break;
            case PoofsJoysticks::RightBumper:  // Quickturn
                if (pressedP) {
                }
                break;
        }
    }
}

void Teleop::HandleXboxJoystick(uint32_t port, uint32_t button, bool pressedP) {
    if (port == OPERATOR_JOYSTICK_PORT) {
        switch (button) {
            case Xbox::BtnY:  // High Elevator Preset
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::Cargo:  // High Rocket Cargo Preset
                            m_elevator->SetPosition(
                                Elevator::HIGH_ROCKET_CARGO);
                            break;
                        case GameMode::Hatch:  // High Rocket Hatch Preset
                            m_elevator->SetPosition(
                                Elevator::HIGH_ROCKET_HATCH);
                            break;
                        case GameMode::EndGame:
                            // Task
                            break;
                    }
                }
                else {
                }
                break;
            case Xbox::BtnA:  // Low Preset
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::Cargo:  // Low Rocket Cargo Preset
                            m_elevator->SetPosition(Elevator::LOW_ROCKET_CARGO);
                            break;
                        case GameMode::Hatch:  // Low Rocket Hatch Preset
                            m_elevator->SetPosition(Elevator::LOW_ROCKET_HATCH);
                            break;
                        case GameMode::EndGame:
                            // Task
                            break;
                    }
                }
                else {
                }
                break;
            case Xbox::BtnX:  // Cargo Bay Preset
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::Cargo:  // Cargo Bay Cargo Preset
                            m_elevator->SetPosition(Elevator::CARGO_SHIP_CARGO);
                            break;
                        case GameMode::Hatch:  // Cargo Bay Hatch Preset
                            m_elevator->SetPosition(Elevator::CARGO_SHIP_HATCH);
                            break;
                        case GameMode::EndGame:
                            // Task
                            break;
                    }
                }
                else {
                }
                break;
            case Xbox::BtnB:  // Middle Elevator Preset
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::Cargo:  // Middle Rocket Cargo Preset
                            m_elevator->SetPosition(
                                Elevator::MIDDLE_ROCKET_CARGO);
                            break;
                        case GameMode::Hatch:  // Middle Rocket Hatch Preset
                            m_elevator->SetPosition(
                                Elevator::MIDDLE_ROCKET_HATCH);
                            break;
                        case GameMode::EndGame:
                            // Task
                            break;
                    }
                }
                else {
                }
                break;
            case Xbox::LeftBumper:  // Extend Intake
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::Cargo:
                            m_cargoIntake->GoToWristState(
                                CargoIntake::CargoWristState::extended);
                            break;
                        case GameMode::Hatch:
                            m_hatchIntake->ManualPuncherActivate();
                            break;
                        case GameMode::EndGame:
                            // Task
                            break;
                    }
                }
                else {
                }
                break;
            case Xbox::RightBumper:  // Intake
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::Cargo:
                            m_cargoIntake->RunIntake();
                            m_elevator->SetPosition(Elevator::GROUND);
                            break;
                        case GameMode::Hatch:
                            m_hatchIntake->RunIntake();
                            m_elevator->SetPosition(Elevator::GROUND);
                            break;
                        case GameMode::EndGame:
                            // Task
                            break;
                    }
                }
                else {
                    switch (m_gameMode) {
                        case GameMode::Cargo:
                            m_cargoIntake->HoldCargo();
                            break;
                        case GameMode::Hatch:
                            m_hatchIntake->HoldHatch();
                            break;
                        case GameMode::EndGame:
                            // Task
                            break;
                    }
                }
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
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::Cargo:
                            // Task
                            break;
                        case GameMode::Hatch:
                            // Task
                            break;
                        case GameMode::EndGame:
                            // Task
                            break;
                    }
                }
                else {
                }
                break;
            case Xbox::DPadLeftVirtBtn:  // Changes game mode to Cargo
                if (pressedP) {
                    m_gameMode = GameMode::Cargo;
                    m_hatchIntake->SetIdle();
                    m_hatchIntake->ManualPuncherRetract();
                    m_rumble = Rumble::on;
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
