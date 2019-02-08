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
    switch (button) {
        case Xbox::BtnY:  // High Elevator Preset
            if (pressedP) {
                return Elevator::HIGH_ROCKET_CARGO;
            }
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
        case Xbox::BtnB:  // Middle Elevator Preset
            if (pressedP) {
                return Elevator::MIDDLE_ROCKET_CARGO;
            }
            break;
    }
}

double PresetHandlerDispatcher::GetHatchPresetFromButton(uint32_t button,
                                                         bool pressedP) {
    switch (button) {
        case Xbox::BtnY:  // High Elevator Preset
            if (pressedP) {
                return Elevator::HIGH_ROCKET_HATCH;
            }
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
        case Xbox::BtnB:  // Middle Elevator Preset
            if (pressedP) {
                return Elevator::MIDDLE_ROCKET_HATCH;
            }
            break;
    }
}

double PresetHandlerDispatcher::GetEndGamePresetFromButton(uint32_t button,
                                                           bool pressedP) {
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
}

void PresetHandlerDispatcher::ElevatorDispatchPressedButtonToPreset(
    Autonomous *mode, uint32_t button, bool pressedP) {
    switch (mode->m_gameMode) {
        case GameMode::Cargo:
            mode->m_elevator->SetPosition(
                GetCargoPresetFromButton(button, pressedP));
            break;
        case GameMode::Hatch:
            mode->m_elevator->SetPosition(
                GetHatchPresetFromButton(button, pressedP));
            break;
        case GameMode::EndGame:
            mode->m_elevator->SetPosition(
                GetEndGamePresetFromButton(button, pressedP));
            break;
        default:
            break;
    }
}

void PresetHandlerDispatcher::ElevatorDispatchPressedButtonToPreset(
    Teleop *mode, uint32_t button, bool pressedP) {
    switch (mode->m_gameMode) {
        case GameMode::Cargo:
            mode->m_elevator->SetPosition(
                GetCargoPresetFromButton(button, pressedP));
            break;
        case GameMode::Hatch:
            mode->m_elevator->SetPosition(
                GetHatchPresetFromButton(button, pressedP));
            break;
        case GameMode::EndGame:
            mode->m_elevator->SetPosition(
                GetEndGamePresetFromButton(button, pressedP));
            break;
        default:
            break;
    }
}

void PresetHandlerDispatcher::ElevatorDispatchPressedButtonToPreset(
    Test *mode, uint32_t button, bool pressedP) {
    switch (mode->m_gameMode) {
        case GameMode::Cargo:
            mode->m_elevator->SetPosition(
                GetCargoPresetFromButton(button, pressedP));
            break;
        case GameMode::Hatch:
            mode->m_elevator->SetPosition(
                GetHatchPresetFromButton(button, pressedP));
            break;
        case GameMode::EndGame:
            mode->m_elevator->SetPosition(
                GetEndGamePresetFromButton(button, pressedP));
            break;
        default:
            break;
    }
}

void PresetHandlerDispatcher::PresetPeriodic(Teleop *mode) {
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
