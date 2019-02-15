/*
 * PresetHandlerDispatcher.h
 *
 *  Created on: February 5, 2019
 *      Authors: John P, Luis, Dylan
 */
#pragma once

#include "frc/WPILib.h"

#define NO_PRESET_NO_CHANGE -1

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
    void ElevatorDispatchPressedButtonToPreset(Autonomous *mode,
                                               uint32_t button, bool pressedP);
    /**
     * Select and use the elevator preset for Teleop "mode", dispatched by the
     * GameMode (Cargo, Hartch, EndGame).
     */
    void ElevatorDispatchPressedButtonToPreset(Teleop *mode, uint32_t button,
                                               bool pressedP);
    /**
     * Select and use the elevator preset for Test "mode", dispatched by
     * the GameMode (Cargo, Hartch, EndGame).
     */
    void ElevatorDispatchPressedButtonToPreset(Test *mode, uint32_t button,
                                               bool pressedP);
    /**
     *  Select and use the driver dispatch joystick button by the GameMode
     * (Cargo, Hatch, EndGame)
     */
    void DriveDispatchJoystickTrigger(Teleop *mode, uint32_t button,
                                      bool pressedP);

    void DriveDispatchJoystickTrigger(Autonomous *mode, uint32_t button,
                                      bool pressedP);

    void DriveDispatchJoystickBumper(Teleop *mode, uint32_t button,
                                     bool pressedP);

    void DriveDispatchJoystickBumper(Autonomous *mode, uint32_t button,
                                     bool pressedP);

    void JoystickPeriodic(Teleop *mode);

    void IntakeBumperPresets(Teleop *mode, uint32_t button, bool pressedP);

private:
    double GetCargoPresetFromButton(uint32_t button, bool pressedP);
    double GetHatchPresetFromButton(uint32_t button, bool pressedP);
    double GetEndGamePresetFromButton(uint32_t button, bool pressedP);
};
}
