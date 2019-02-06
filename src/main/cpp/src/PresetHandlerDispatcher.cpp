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

void PresetHandlerDispatcher::DispatchPressedButtonToPreset(Autonomous *mode,
                                                            uint32_t button,
                                                            bool pressedP) {
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

void PresetHandlerDispatcher::DispatchPressedButtonToPreset(Teleop *mode,
                                                            uint32_t button,
                                                            bool pressedP) {
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

void PresetHandlerDispatcher::DispatchPressedButtonToPreset(Test *mode,
                                                            uint32_t button,
                                                            bool pressedP) {
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
}
