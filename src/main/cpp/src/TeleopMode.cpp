/*
 * TeleopMode.cpp
 *
 *  Created on: January 7, 2019
 *      Authors: Kyle, Chris
 *
 */
#include "src/TeleopMode.h"

namespace frc973 {
Teleop::Teleop(ObservablePoofsJoystick *driverJoystick,
               ObservableXboxJoystick *operatorJoystick,
               ObservableDualActionJoystick *tuningJoystick, Drive *drive,
               Elevator *elevator, HatchIntake *hatchIntake,
               CargoIntake *cargoIntake, Stinger *stinger,
               Limelight *limelightHatch)
        : m_driverJoystick(driverJoystick)
        , m_operatorJoystick(operatorJoystick)
        , m_tuningJoystick(tuningJoystick)
        , m_drive(drive)
        , m_driveMode(DriveMode::Cheesy)
        , m_elevator(elevator)
        , m_hatchIntake(hatchIntake)
        , m_cargoIntake(cargoIntake)
        , m_stinger(stinger)
        , m_limelightHatch(limelightHatch)
        , m_rumble(Rumble::off)
        , m_gameMode(GameMode::HatchInit) {
}

Teleop::~Teleop() {
}

void Teleop::TeleopInit() {
    std::cout << "Teleop Start" << std::endl;
    m_elevator->EnableCoastMode();
    m_cargoIntake->EnableCoastMode();
    m_drive->EnableCoastMode();
    m_stinger->RetractSwitchBlade();
    m_stinger->SetKickUpDisable();
    m_driveMode = DriveMode::Cheesy;
    m_drive->Zero();
}

void Teleop::TeleopPeriodic() {
    /**
     * Driver Joystick
     */
    double y =  // m_operatorJoystick->GetRawAxisWithDeadband(Xbox::LeftYAxis);
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoystick::LeftYAxis);
    double x =  // m_operatorJoystick->GetRawAxisWithDeadband(Xbox::RightXAxis);
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoystick::RightXAxis);
    bool quickturn = m_driverJoystick->GetRawButton(PoofsJoystick::RightBumper);
    bool softwareLowGear =
        m_driverJoystick->GetRawButton(PoofsJoystick::RightTrigger);

    DBStringPrintf(DB_LINE7, "td:%2.2lf xo:%2.2lf s:%2.2lf",
                   m_limelightHatch->GetHorizontalDistance(),
                   m_limelightHatch->GetXOffset(),
                   m_limelightHatch->GetTargetSkew());

    switch (m_driveMode) {
        case DriveMode::Cheesy:
            DBStringPrintf(DB_LINE2, "Drive Mode: Cheesy Drive");
            if (softwareLowGear) {
                m_drive->CheesyDrive(y / 3.0, x / 3.0, quickturn, true);
            }
            else {
                m_drive->CheesyDrive(y, x, quickturn, true);
            }
            break;
        case DriveMode::Openloop:
            DBStringPrintf(DB_LINE2, "Drive Open Loop");
            if (softwareLowGear) {
                m_drive->OpenloopArcadeDrive(y / 3.0, x / 3.0);
            }
            else {
                m_drive->OpenloopArcadeDrive(y, x);
            }
            break;
        case DriveMode::LimelightDriveWithSkew:
            DBStringPrintf(DB_LINE2, "Drive lime with skew");
            m_drive->LimelightDriveWithSkew();
            break;
        case DriveMode::LimelightDriveWithoutSkew:
            DBStringPrintf(DB_LINE2, "Drive lime no skew");
            m_drive->LimelightDriveWithoutSkew();
            break;
        case DriveMode::AssistedCheesyHatch:
            DBStringPrintf(DB_LINE2, "Drive Assisted Cheesy");
            m_drive->AssistedCheesyHatchDrive(y, x, quickturn, false);
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
            if (GetMsecTime() - m_wristResetTimer > 500) {
                m_cargoIntake->RetractWrist();
            }
            break;
        case GameMode::HatchInit:
            m_cargoIntake->RetractPlatformWheel();
            m_cargoIntake->StopIntake();
            m_wristResetTimer = GetMsecTime();
            m_gameMode = GameMode::HatchPeriodic;
            break;
        case GameMode::HatchPeriodic:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: hatch");
            if (GetMsecTime() - m_wristResetTimer > 500) {
                m_cargoIntake->RetractWrist();
            }
            break;
        case GameMode::ThirdLevelEndGameInit:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: ThirdLevelEndGameinit");
            m_limelightHatch->SetCameraDriver();
            m_cargoIntake->StopIntake();
            m_hatchIntake->SetIdle();
            m_elevator->SetPosition(Elevator::THIRD_PLATFORM);
            if (m_elevator->GetPosition() > Elevator::THIRD_PLATFORM - 1.0) {
                m_cargoIntake->DeployPlatformWheel();
                m_cargoIntake->RetractWrist();
                m_gameMode = GameMode::ThirdLevelEndGamePeriodic;
            }
            break;
        case GameMode::SecondLevelEndGameInit:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: SecondLevelEndGameinit");
            m_limelightHatch->SetCameraDriver();
            m_cargoIntake->StopIntake();
            m_hatchIntake->SetIdle();
            m_elevator->SetPosition(Elevator::SECOND_PLATFORM);
            if (m_elevator->GetPosition() > Elevator::SECOND_PLATFORM - 2.0) {
                m_cargoIntake->DeployPlatformWheel();
                m_cargoIntake->RetractWrist();
                m_gameMode = GameMode::SecondLevelEndGamePeriodic;
            }
            break;
        case GameMode::ThirdLevelEndGamePeriodic:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: 3endgameperiodic");
            m_drive->SetStingerOutput(y);
            m_driveMode = DriveMode::Cheesy;
            if (m_driverJoystick->GetRawButton(PoofsJoystick::LeftTrigger)) {
                m_elevator->SetPosition(12.0);
                m_cargoIntake->RetractPlatformWheel();
            }
            break;
        case GameMode::SecondLevelEndGamePeriodic:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: 2endgameperiodic");
            m_drive->SetStingerOutput(y);
            m_driveMode = DriveMode::Cheesy;
            break;
        case GameMode::SecondLevelStabilize:
            if (m_elevator->GetPosition() > 21.0) {
                m_elevator->SetPosition(14.0);
                m_gameMode = GameMode::SecondLevelEndGamePeriodic;
            }
            break;
        case GameMode::RaiseIntake:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: raiseintake");
            m_elevator->SetPosition(Elevator::THIRD_PLATFORM_RESET);
            m_cargoIntake->RetractPlatformWheel();
            m_gameMode = GameMode::ResetIntake;
            break;
        case GameMode::ResetIntake:
            DBStringPrintf(DBStringPos::DB_LINE8, "gm: resetintake");
            if (m_elevator->GetPosition() >
                Elevator::THIRD_PLATFORM_RESET - 2.0) {
                m_cargoIntake->RetractWrist();
                m_elevator->SetPosition(0.4);
                m_gameMode = GameMode::ThirdLevelEndGamePeriodic;
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
            if ((GetMsecTime() - m_rumbleTimer) > 500) {
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
            case GameMode::ThirdLevelEndGamePeriodic:
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
            case PoofsJoystick::LeftTrigger:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_driveMode = DriveMode::LimelightDriveWithSkew;
                            break;
                        case GameMode::CargoPeriodic:
                            break;
                        case GameMode::ThirdLevelEndGamePeriodic:

                            break;
                        case GameMode::SecondLevelEndGamePeriodic:
                            break;
                    }
                }
                else {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_driveMode = DriveMode::Cheesy;
                            break;
                        case GameMode::CargoPeriodic:
                            break;
                        case GameMode::ThirdLevelEndGamePeriodic:
                            break;
                        case GameMode::SecondLevelEndGamePeriodic:
                            break;
                    }
                }
                break;
            case PoofsJoystick::RightTrigger:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_hatchIntake->Exhaust();
                            break;
                        case GameMode::CargoPeriodic:
                            m_cargoIntake->Exhaust();
                            break;
                        case GameMode::
                            ThirdLevelEndGamePeriodic:  // we want the same
                                                        // action for both
                                                        // buttons
                        case GameMode::SecondLevelEndGamePeriodic:
                            if (m_stinger->GetSwitchBladeState() ==
                                Stinger::SwitchBladeState::retracted) {
                                m_gameMode = GameMode::RaiseIntake;
                            }
                            else if (m_stinger->GetSwitchBladeState() ==
                                     Stinger::SwitchBladeState::engaged) {
                                m_stinger->SetKickUpEnable();
                            }
                            break;
                    }
                }
                else {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_hatchIntake->SetIdle();
                            break;
                        case GameMode::CargoPeriodic:
                            m_cargoIntake->StopIntake();
                            break;
                        case GameMode::ThirdLevelEndGamePeriodic:
                        case GameMode::SecondLevelEndGamePeriodic:
                            break;
                    }
                }
                break;
            case PoofsJoystick::LeftBumper:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_driveMode = DriveMode::LimelightDriveWithoutSkew;
                            break;
                        case GameMode::CargoPeriodic:
                            break;
                        case GameMode::
                            ThirdLevelEndGamePeriodic:  // we want the same for
                                                        // both buttons
                        case GameMode::SecondLevelEndGamePeriodic:
                            m_elevator->SetPower(
                                -ELEVATOR_STINGER_VOLTAGE_RATIO);
                            break;
                    }
                }
                else {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_driveMode = DriveMode::Cheesy;
                            break;
                        case GameMode::CargoPeriodic:
                            break;
                        case GameMode::ThirdLevelEndGamePeriodic:
                        case GameMode::SecondLevelEndGamePeriodic:
                            m_elevator->SetPower(0.0);
                            break;
                    }
                }
                break;
            case PoofsJoystick::RightBumper:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::SecondLevelEndGamePeriodic:
                            m_elevator->SetPosition(23.0);
                            m_cargoIntake->DeployPlatformWheel();
                            m_gameMode = GameMode::SecondLevelStabilize;
                            break;
                    }
                }
                break;
        }
    }
}

void Teleop::HandleXboxJoystick(uint32_t port, uint32_t button, bool pressedP) {
    if (port == OPERATOR_JOYSTICK_PORT) {
        switch (button) {
            case Xbox::Back:
                if (pressedP) {
                    m_gameMode = GameMode::ThirdLevelEndGameInit;
                    m_rumble = Rumble::on;
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case Xbox::Start:
                if (pressedP) {
                    m_gameMode = GameMode::SecondLevelEndGameInit;
                    m_rumble = Rumble::on;
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case Xbox::DPadUpVirtBtn:
                if (pressedP) {
                }
                break;
            case Xbox::DPadDownVirtBtn:
                break;
            case Xbox::DPadLeftVirtBtn:
                if (pressedP) {
                    m_gameMode = GameMode::CargoInit;
                    m_rumble = Rumble::on;
                    m_limelightHatch->SetCameraDriver();
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case Xbox::DPadRightVirtBtn:
                if (pressedP) {
                    m_gameMode = GameMode::HatchInit;
                    m_rumble = Rumble::on;
                    m_limelightHatch->SetCameraDriver();
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case Xbox::BtnY:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_elevator->SetPosition(Elevator::MID_ROCKET_HATCH);
                            m_elevator->SetRocketScoreMode(
                                Elevator::RocketScoreMode::middle);
                            break;
                        case GameMode::CargoPeriodic:
                            m_elevator->SetPosition(Elevator::MID_ROCKET_CARGO);
                            break;
                        case GameMode::ThirdLevelEndGamePeriodic:
                            m_stinger->SetKickUpEnable();
                            break;
                    }
                }
                break;
            case Xbox::BtnA:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_elevator->SetPosition(Elevator::LOW_ROCKET_HATCH);
                            m_elevator->SetRocketScoreMode(
                                Elevator::RocketScoreMode::low);
                            break;
                        case GameMode::CargoPeriodic:
                            m_elevator->SetPosition(Elevator::LOW_ROCKET_CARGO);
                            break;
                        case GameMode::ThirdLevelEndGamePeriodic:
                            m_stinger->SetKickUpDisable();
                            break;
                    }
                }
                break;
            case Xbox::BtnX:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_elevator->SetPosition(Elevator::CARGO_SHIP_HATCH);
                            break;
                        case GameMode::CargoPeriodic:
                            m_elevator->SetPosition(Elevator::CARGO_SHIP_CARGO);
                            break;
                        case GameMode::ThirdLevelEndGamePeriodic:
                            m_stinger->RetractSwitchBlade();
                            m_stinger->RetractGateLatch();
                            break;
                    }
                }
                break;
            case Xbox::LeftBumper:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_hatchIntake->ManualPuncherActivate();
                            break;
                        case GameMode::CargoPeriodic:
                            m_cargoIntake->ExtendWrist();
                            break;
                    }
                }
                break;
            case Xbox::RightBumper:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_hatchIntake->RunIntake();
                            m_elevator->SetPosition(Elevator::GROUND);
                            m_elevator->SetRocketScoreMode(
                                Elevator::RocketScoreMode::low);
                            break;
                        case GameMode::CargoPeriodic:
                            m_cargoIntake->RunIntake();
                            m_elevator->SetPosition(Elevator::GROUND);
                            break;
                    }
                }
                else {
                    switch (m_gameMode) {
                        case GameMode::HatchPeriodic:
                            m_hatchIntake->HoldHatch();
                            m_limelightHatch->SetLightOff();
                            break;
                        case GameMode::CargoPeriodic:
                            m_cargoIntake->HoldCargo();
                            m_limelightHatch->SetLightOff();
                            break;
                    }
                }
                break;
            case Xbox::BtnB:
                if (pressedP) {
                    switch (m_gameMode) {
                        case GameMode::CargoPeriodic:
                            m_cargoIntake->RunIntake(1.0);
                            m_elevator->SetPosition(
                                Elevator::LOADING_STATION_CARGO);
                            break;
                        case GameMode::ThirdLevelEndGamePeriodic:
                            m_stinger->DeploySwitchBlade();
                            m_stinger->EngageGateLatch();
                            break;
                        case GameMode::HatchPeriodic:
                            m_elevator->SetPosition(Elevator::GROUND);
                            m_elevator->SetRocketScoreMode(
                                Elevator::RocketScoreMode::middle);
                            break;
                    }
                }
                else {
                    switch (m_gameMode) {
                        case GameMode::CargoPeriodic:
                            m_cargoIntake->HoldCargo();
                            break;
                    }
                }
                break;
            case Xbox::LJoystickBtn:
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

Teleop::GameMode Teleop::GetGameMode() {
    return m_gameMode;
}
}
