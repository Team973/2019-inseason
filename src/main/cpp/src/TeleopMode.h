/*
 * TeleopMode.h
 *
 *  Created on: January 7, 2018
 *      Authors: Kyle, Chris
 */
#pragma once

#include "frc/WPILib.h"
#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"
#include "lib/sensors/Limelight.h"
#include "lib/pixelprocessors/Flash.h"
#include "lib/util/WrapDash.h"
#include "src/info/RobotInfo.h"
#include "src/subsystems/Elevator.h"
#include "src/subsystems/Drive.h"
#include "src/subsystems/CargoIntake.h"
#include "src/subsystems/HatchIntake.h"
#include "src/subsystems/Stinger.h"
#include <iostream>

using namespace frc;

namespace frc973 {

/**
 * Controls the teleop mode.
 */
class Teleop {
public:
    /**
     * Constuct a teleop mode.
     * @param driver The driver's joystick.
     * @param codriver The co-driver's joystick.
     * @param drive The drive subsystem.
     * @param limelightCargo The Limelight for the cargo.
     * @param limelightHatch The Limelight for the hatch.
     */
    Teleop(ObservablePoofsJoystick *driver, ObservableXboxJoystick *codriver,
           Drive *drive, Elevator *elevator, HatchIntake *hatchintake,
           CargoIntake *cargoIntake, Stinger *stinger,
           Limelight *limelightCargo, Limelight *limelightHatch);

    virtual ~Teleop();

    /**
     * Start of teleop.
     */
    void TeleopInit();

    /**
     * Loop of teleop.
     */
    void TeleopPeriodic();

    /**
     * Stop of teleop.
     */
    void TeleopStop();

    /**
     * Button handler for the disabled mode.
     * @param port The port the joystick is connected to.
     * @param button The button.
     * @param pressedP The button's new status.
     */
    void HandleDualActionJoystick(uint32_t port, uint32_t button,
                                  bool pressedP);

    /**
     * Button handler for the disabled mode.
     * @param port The port the joystick is connected to.
     * @param button The button.
     * @param pressedP The button's new status.
     */
    void HandlePoofsJoystick(uint32_t port, uint32_t button, bool pressedP);

    /**
     * Button handler for the disabled mode.
     * @param port The port the joystick is connected to.
     * @param button The button.
     * @param pressedP The button's new status.
     */
    void HandleXboxJoystick(uint32_t port, uint32_t button, bool pressedP);
    static constexpr double ELEVATOR_STINGER_VOLTAGE_RATIO = 1.0;

private:
    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;

    Drive *m_drive;
    enum class DriveMode
    {
        Openloop,
        LimelightCargo,
        LimelightHatch,
        AssistedCheesy,
        Cheesy
    };
    DriveMode m_driveMode;
    CargoIntake *m_cargoIntake;

    enum class GameMode
    {
        Cargo,
        Hatch,
        EndGameInit,
        EndGamePeriodic,
        RaiseIntake,
        ResetIntake
    };
    GameMode m_gameMode;

    HatchIntake *m_hatchIntake;
    Elevator *m_elevator;
    Stinger *m_stinger;

    Limelight *m_limelightCargo;
    Limelight *m_limelightHatch;
    enum class Rumble
    {
        on,
        off
    };
    Rumble m_rumble;

    uint32_t m_rumbleTimer;
    uint32_t m_limelightCargoTimer;
    uint32_t m_limelightHatchTimer;
};
}
