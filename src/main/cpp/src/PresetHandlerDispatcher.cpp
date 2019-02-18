/*
 * PresetHandlerDispatcher.h
 *
 *  Created on: February 5, 2019
 *      Authors: John P, Luis, Dylan
 */
#include "src/PresetHandlerDispatcher.h"
#include "src/GameMode.h"
#include "src/AutonomousMode.h"
#include "src/TeleopMode.h"
#include "src/TestMode.h"

using namespace frc;

namespace frc973 {

double PresetHandlerDispatcher::GetCargoPresetFromButton(uint32_t button,
                                                         bool pressedP) {
    double ret = NO_PRESET_NO_CHANGE;

    switch (button) {
        case Xbox::BtnA:  // Low Preset
            if (pressedP) {
                return Elevator::LOW_ROCKET_CARGO;
            }
            break;
        case Xbox::BtnX:  // Cargo Bay Preset
            if (pressedP) {
                return Elevator::CARGO_SHIP_CARGO;
            }
            break;
        case Xbox::BtnB:
            if (pressedP) {
                return Elevator::LOADING_STATION_CARGO;
            }
            break;
        case Xbox::BtnY:
            if (pressedP) {
                return Elevator::MID_ROCKET_CARGO;
            }
            break;
    }

    return ret;
}

double PresetHandlerDispatcher::GetHatchPresetFromButton(uint32_t button,
                                                         bool pressedP) {
    double ret = NO_PRESET_NO_CHANGE;

    switch (button) {
        case Xbox::BtnA:  // Low Preset
            if (pressedP) {
                return Elevator::LOW_ROCKET_HATCH;
            }
            break;
        case Xbox::BtnX:  // Cargo Bay Preset
            if (pressedP) {
                return Elevator::CARGO_SHIP_HATCH;
            }
            break;
        case Xbox::BtnB:
            break;
        case Xbox::BtnY:
            if (pressedP) {
                return Elevator::MID_ROCKET_CARGO;
            }
            break;
    }

    return ret;
}

double PresetHandlerDispatcher::GetEndGamePresetFromButton(uint32_t button,
                                                           bool pressedP) {
    double ret = NO_PRESET_NO_CHANGE;

    switch (button) {
        case Xbox::BtnA:  // Low Preset
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::BtnX:  // Cargo Bay Preset
            if (pressedP) {
            }
            else {
            }
            break;
        case Xbox::BtnB:  // Middle Elevator Preset
            if (pressedP) {
            }
            else {
            }
            break;
    }

    return ret;
}

void PresetHandlerDispatcher::ElevatorDispatchPressedButtonToPreset(
    Teleop *mode, uint32_t button, bool pressedP) {
    double height = NO_PRESET_NO_CHANGE;

    switch (mode->m_gameMode) {
        case GameMode::Cargo:
            height = GetCargoPresetFromButton(button, pressedP);
            break;
        case GameMode::Hatch:
            height = GetHatchPresetFromButton(button, pressedP);
            break;
        case GameMode::EndGamePeriodic:
            height = GetEndGamePresetFromButton(button, pressedP);
            break;
        default:
            break;
    }

    if (height != NO_PRESET_NO_CHANGE) {
        mode->m_elevator->SetPosition(height);
    }
}

void PresetHandlerDispatcher::ElevatorDispatchPressedButtonToPreset(
    Test *mode, uint32_t button, bool pressedP) {
    double height = NO_PRESET_NO_CHANGE;

    switch (mode->m_gameMode) {
        case GameMode::Cargo:
            height = GetCargoPresetFromButton(button, pressedP);
            break;
        case GameMode::Hatch:
            height = GetHatchPresetFromButton(button, pressedP);
            break;
        case GameMode::EndGamePeriodic:
            height = GetEndGamePresetFromButton(button, pressedP);
            break;
        default:
            break;
    }

    if (height != NO_PRESET_NO_CHANGE) {
        mode->m_elevator->SetPosition(height);
    }
}

void PresetHandlerDispatcher::DriveDispatchJoystickButtons(Teleop *mode,
                                                           uint32_t button,
                                                           bool pressedP) {
    switch (mode->m_gameMode) {
        case GameMode::Cargo:
            if (pressedP) {
                switch (button) {
                    case PoofsJoysticks::LeftTrigger:
                        mode->m_driveMode =
                            Teleop::DriveMode::AssistedCheesyCargo;
                        break;
                    case PoofsJoysticks::RightTrigger:
                        mode->m_cargoIntake->Exhaust();
                        break;
                    case PoofsJoysticks::LeftBumper:
                        mode->m_driveMode = Teleop::DriveMode::LimelightCargo;
                        break;
                    case PoofsJoysticks::RightBumper:
                        break;
                }
            }
            else {
                switch (button) {
                    case PoofsJoysticks::LeftTrigger:
                        mode->m_driveMode = Teleop::DriveMode::Cheesy;
                        break;
                    case PoofsJoysticks::RightTrigger:
                        mode->m_cargoIntake->StopIntake();
                        break;
                    case PoofsJoysticks::LeftBumper:
                        mode->m_driveMode = Teleop::DriveMode::Cheesy;
                        break;
                    case PoofsJoysticks::RightBumper:
                        break;
                }
            }
            break;
        case GameMode::Hatch:
            if (pressedP) {
                switch (button) {
                    case PoofsJoysticks::LeftTrigger:
                        mode->m_driveMode =
                            Teleop::DriveMode::AssistedCheesyHatch;
                        break;
                    case PoofsJoysticks::RightTrigger:
                        mode->m_hatchIntake->Exhaust();
                        break;
                    case PoofsJoysticks::LeftBumper:
                        mode->m_driveMode = Teleop::DriveMode::LimelightHatch;
                        break;
                    case PoofsJoysticks::RightBumper:
                        break;
                }
            }
            else {
                switch (button) {
                    case PoofsJoysticks::LeftTrigger:
                        mode->m_driveMode = Teleop::DriveMode::Cheesy;
                        break;
                    case PoofsJoysticks::RightTrigger:
                        mode->m_hatchIntake->SetIdle();
                        break;
                    case PoofsJoysticks::LeftBumper:
                        mode->m_driveMode = Teleop::DriveMode::Cheesy;
                        break;
                    case PoofsJoysticks::RightBumper:
                        break;
                }
            }
            break;
        case GameMode::EndGamePeriodic:
            if (pressedP) {
                switch (button) {
                    case PoofsJoysticks::LeftTrigger:
                        if (mode->m_cargoIntake->GetWristState() ==
                            CargoIntake::CargoWristState::extended) {
                            mode->m_elevator->SetPower(
                                Teleop::ELEVATOR_STINGER_VOLTAGE_RATIO * 0.6);
                            mode->m_stinger->SetPower(0.8);
                        }
                        else if (mode->m_cargoIntake->GetWristState() ==
                                 CargoIntake::CargoWristState::retracted) {
                            mode->m_stinger->SetPower(0.6);
                        }
                        break;
                    case PoofsJoysticks::RightTrigger:
                        mode->m_gameMode = GameMode::RaiseIntake;
                        break;
                    case PoofsJoysticks::LeftBumper:
                        mode->m_elevator->SetPower(
                            -Teleop::ELEVATOR_STINGER_VOLTAGE_RATIO);
                        mode->m_stinger->SetPower(-1.0);
                        break;
                    case PoofsJoysticks::RightBumper:
                        break;
                }
            }
            else {
                switch (button) {
                    case PoofsJoysticks::LeftTrigger:
                        if (mode->m_cargoIntake->GetWristState() ==
                            CargoIntake::CargoWristState::extended) {
                            mode->m_elevator->SetPower(0.0);
                            mode->m_stinger->SetPower(0.0);
                        }
                        else if (mode->m_cargoIntake->GetWristState() ==
                                 CargoIntake::CargoWristState::retracted) {
                            mode->m_stinger->SetPower(0.0);
                        }
                        break;
                    case PoofsJoysticks::RightTrigger:
                        break;
                    case PoofsJoysticks::LeftBumper:
                        mode->m_elevator->SetPower(0.0);
                        mode->m_stinger->SetPower(0.0);
                        break;
                    case PoofsJoysticks::RightBumper:
                        break;
                }
            }
            break;
        default:
            break;
    }
}

void PresetHandlerDispatcher::IntakeBumperPresets(Teleop *mode, uint32_t button,
                                                  bool pressedP) {
    switch (mode->m_gameMode) {
        case GameMode::Cargo:
            if (pressedP) {
                if (button == Xbox::LeftBumper) {
                    mode->m_cargoIntake->GoToWristState(
                        CargoIntake::CargoWristState::extended);
                }
                else if (button ==
                         Xbox::RightBumper) {  // button == Xbox::RightBumper
                    mode->m_cargoIntake->RunIntake();
                    mode->m_elevator->SetPosition(Elevator::GROUND);
                }
                else {
                    mode->m_elevator->SetPosition(
                        Elevator::LOADING_STATION_CARGO);
                    mode->m_cargoIntake->RunIntake(1.0);
                }
            }
            else {
                if (button == Xbox::RightBumper || button == Xbox::BtnB) {
                    mode->m_cargoIntake->HoldCargo();
                }
            }

            break;
        case GameMode::Hatch:
            if (pressedP) {
                if (button == Xbox::LeftBumper) {
                    mode->m_hatchIntake->ManualPuncherActivate();
                }
                else if (button ==
                         Xbox::RightBumper) {  // button == Xbox::RightBumper
                    mode->m_hatchIntake->RunIntake();
                    mode->m_elevator->SetPosition(Elevator::GROUND);
                }
            }
            else {
                if (button == Xbox::RightBumper) {
                    mode->m_hatchIntake->HoldHatch();
                }
            }
            break;
        case GameMode::EndGamePeriodic:
            break;
    }
}
}
