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
        case Xbox::BtnY:
            break;
        case Xbox::BtnA:  // Low Preset
            if (pressedP) {
                return Elevator::LOW_ROCKET_CARGO;
            }
        case Xbox::BtnX:  // Cargo Bay Preset
            if (pressedP) {
                return Elevator::CARGO_SHIP_CARGO;
            }
            break;
        case Xbox::BtnB:
            break;
    }

    return ret;
}

double PresetHandlerDispatcher::GetHatchPresetFromButton(uint32_t button,
                                                         bool pressedP) {
    double ret = NO_PRESET_NO_CHANGE;

    switch (button) {
        case Xbox::BtnY:
            break;
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
    }

    return ret;
}

double PresetHandlerDispatcher::GetEndGamePresetFromButton(uint32_t button,
                                                           bool pressedP) {
    double ret = NO_PRESET_NO_CHANGE;

    switch (button) {
        case Xbox::BtnY:  // High Elevator Preset
            if (pressedP) {
            }
            else {
            }
            break;
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
    Autonomous *mode, uint32_t button, bool pressedP) {
    double height = NO_PRESET_NO_CHANGE;

    switch (mode->m_gameMode) {
        case GameMode::Cargo:
            height = GetCargoPresetFromButton(button, pressedP);
            break;
        case GameMode::Hatch:
            height = GetHatchPresetFromButton(button, pressedP);
            break;
        case GameMode::EndGame:
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
    Teleop *mode, uint32_t button, bool pressedP) {
    double height = NO_PRESET_NO_CHANGE;

    switch (mode->m_gameMode) {
        case GameMode::Cargo:
            height = GetCargoPresetFromButton(button, pressedP);
            break;
        case GameMode::Hatch:
            height = GetHatchPresetFromButton(button, pressedP);
            break;
        case GameMode::EndGame:
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
        case GameMode::EndGame:
            height = GetEndGamePresetFromButton(button, pressedP);
            break;
        default:
            break;
    }

    if (height != NO_PRESET_NO_CHANGE) {
        mode->m_elevator->SetPosition(height);
    }
}

void PresetHandlerDispatcher::DriveDispatchJoystickTrigger(Autonomous *mode,
                                                           uint32_t button,
                                                           bool pressedP) {
}

void PresetHandlerDispatcher::DriveDispatchJoystickTrigger(Teleop *mode,
                                                           uint32_t button,
                                                           bool pressedP) {
    switch (mode->m_gameMode) {
        case GameMode::Cargo:
            if (pressedP) {
                if (button == PoofsJoysticks::LeftTrigger) {
                    mode->m_driveMode = Teleop::DriveMode::AssistedCheesyCargo;
                }
                else {  // RightTrigger
                    mode->m_cargoIntake->Exhaust();
                }
            }
            else {
                if (button == PoofsJoysticks::LeftTrigger) {
                    mode->m_driveMode = Teleop::DriveMode::Cheesy;
                }
                else {  // RightTrigger
                    mode->m_cargoIntake->StopIntake();
                }
            }
            break;
        case GameMode::Hatch:
            if (pressedP) {
                if (button == PoofsJoysticks::LeftTrigger) {
                    mode->m_driveMode = Teleop::DriveMode::AssistedCheesyHatch;
                }
                else {  // RightTrigger
                    mode->m_hatchIntake->Exhaust();
                }
            }
            else {
                if (button == PoofsJoysticks::LeftTrigger) {
                    mode->m_driveMode = Teleop::DriveMode::Cheesy;
                }
                else {  // RightTrigger
                    mode->m_hatchIntake->SetIdle();
                }
            }
            break;
        case GameMode::EndGame:
            break;
        default:
            break;
    }
}

void PresetHandlerDispatcher::DriveDispatchJoystickBumper(Autonomous *mode,
                                                          uint32_t button,
                                                          bool pressedP) {
}

void PresetHandlerDispatcher::DriveDispatchJoystickBumper(Teleop *mode,
                                                          uint32_t button,
                                                          bool pressedP) {
    switch (mode->m_gameMode) {
        case GameMode::Cargo:
            if (pressedP) {
                if (button == PoofsJoysticks::LeftBumper) {
                    mode->m_driveMode = Teleop::DriveMode::LimelightCargo;
                }
                else {  // RightBumper
                }
            }
            else {
                if (button == PoofsJoysticks::LeftBumper) {
                    mode->m_driveMode = Teleop::DriveMode::Cheesy;
                }
                else {  // RightBumper
                }
            }
            break;
        case GameMode::Hatch:
            if (pressedP) {
                if (button == PoofsJoysticks::LeftBumper) {
                    mode->m_driveMode = Teleop::DriveMode::LimelightHatch;
                }
                else {  // RightBumper
                }
            }
            else {
                if (button == PoofsJoysticks::LeftBumper) {
                    mode->m_driveMode = Teleop::DriveMode::Cheesy;
                }
                else {  // RightBumper
                }
            }
            break;
        case GameMode::EndGame:
            break;
        default:
            break;
    }
}

void PresetHandlerDispatcher::JoystickPeriodic(Teleop *mode) {
    /**
     * Operator Joystick
     */
    if (fabs(mode->m_operatorJoystick->GetRawAxisWithDeadband(
            Xbox::RightYAxis)) > 0.2) {
        mode->m_elevator->SetManualInput();
    }

    if (fabs(mode->m_operatorJoystick->GetRawAxisWithDeadband(
            Xbox::LeftTriggerAxis)) > 0.25) {
        switch (mode->m_gameMode) {
            case GameMode::Cargo:
                mode->m_cargoIntake->GoToWristState(
                    CargoIntake::CargoWristState::retracted);
                break;
            case GameMode::Hatch:
                mode->m_hatchIntake->ManualPuncherRetract();
                break;
            case GameMode::EndGame:
                // Task
                break;
        }
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
                else {  // button == Xbox::RightBumper
                    mode->m_cargoIntake->RunIntake();
                    mode->m_elevator->SetPosition(Elevator::GROUND);
                }
            }
            else {
                if (button == Xbox::RightBumper) {
                    mode->m_cargoIntake->HoldCargo();
                }
            }

            break;
        case GameMode::Hatch:
            if (pressedP) {
                if (button == Xbox::LeftBumper) {
                    mode->m_hatchIntake->ManualPuncherActivate();
                }
                else {  // button == Xbox::RightBumper
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
        case GameMode::EndGame:
            break;
    }
}
}
