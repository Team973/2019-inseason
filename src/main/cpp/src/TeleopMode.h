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
class PresetHandlerDispatcher;

/**
 * Controls the teleop mode.
 */
class Teleop {
public:
    enum class GameMode
    {
        CargoInit,
        CargoPeriodic,
        HatchInit,
        HatchPeriodic,
        ThirdLevelEndGameInit,
        SecondLevelEndGameInit,
        ThirdLevelEndGamePeriodic,
        SecondLevelEndGamePeriodic,
        SecondLevelStabilize,
        RaiseIntake,
        ResetIntake
    };

    /**
     * Constuct a teleop mode.
     * @param driver The driver's joystick.
     * @param codriver The co-driver's joystick.
     * @param drive The drive subsystem.
     * @param elevator The elevator subsystem.
     * @param hatchintake The hatch intake subsystem.
     * @param cargointake The cargo intake subsystem.
     * @param stinger The stinger subsystem.
     * @param limelightHatch The Limelight for the hatch.
     */
    Teleop(ObservablePoofsJoystick *driver, ObservableXboxJoystick *codriver,
           ObservableDualActionJoystick *testStick, Drive *drive,
           Elevator *elevator, HatchIntake *hatchintake,
           CargoIntake *cargoIntake, Stinger *stinger,
           Limelight *limelightHatch);

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

    GameMode GetGameMode();

    static constexpr double ELEVATOR_STINGER_VOLTAGE_RATIO = 1.0;

private:
    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;
    ObservableDualActionJoystick *m_testJoystick;

    Drive *m_drive;
    enum class DriveMode
    {
        Openloop,
        LimelightDriveWithSkew,
        LimelightDriveWithoutSkew,
        AssistedCheesyHatch,
        Cheesy
    };
    DriveMode m_driveMode;

    CargoIntake *m_cargoIntake;
    HatchIntake *m_hatchIntake;
    Elevator *m_elevator;
    Stinger *m_stinger;

    Limelight *m_limelightHatch;

    enum class Rumble
    {
        on,
        off
    };
    Rumble m_rumble;
    GameMode m_gameMode;

    uint32_t m_rumbleTimer;
    uint32_t m_wristResetTimer;
};
}
