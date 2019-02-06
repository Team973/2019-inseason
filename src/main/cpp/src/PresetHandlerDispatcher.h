/*
 * PresetHandlerDispatcher.h
 *
 *  Created on: February 5, 2019
 *      Authors: John P, Luis, Dylan
 */
#pragma once

#include "frc/WPILib.h"

using namespace frc;

namespace frc973 {
class Teleop;
class Test;
class Autonomous;

/**
 * Handles the elevator preset dispatching for the "main" operation mode
 * (autonomous, teleop, test) for the given "GameMode" (Cargo, Harch, and
 * EndGame). These heights for the elevator change per the GameMode state.
 */
class PresetHandlerDispatcher {
public:
    /**
     * Select and use the elevator preset for Autonomouse "mode", dispatched by
     * the GameMode (Cargo, Hartch, EndGame).
     */
    void DispatchPressedButtonToPreset(Autonomous *mode, uint32_t button,
                                       bool pressedP);
    /**
     * Select and use the elevator preset for Teleop "mode", dispatched by the
     * GameMode (Cargo, Hartch, EndGame).
     */
    void DispatchPressedButtonToPreset(Teleop *mode, uint32_t button,
                                       bool pressedP);
    /**
     * Select and use the elevator preset for Test "mode", dispatched by
     * the GameMode (Cargo, Hartch, EndGame).
     */
    void DispatchPressedButtonToPreset(Test *mode, uint32_t button,
                                       bool pressedP);

private:
    double GetCargoPresetFromButton(uint32_t button, bool pressedP);
    double GetHatchPresetFromButton(uint32_t button, bool pressedP);
    double GetEndGamePresetFromButton(uint32_t button, bool pressedP);
};
}
