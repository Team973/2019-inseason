/*
 * TeleopMode.h
 *
 *  Created on: January 7, 2018
 *      Authors: Kyle, Chris
 */
#pragma once

#include "frc/WPILib.h"
#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/GreyLight.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"
#include "lib/sensors/Limelight.h"
#include "lib/pixelprocessors/Flash.h"
#include "lib/util/WrapDash.h"
#include "src/info/RobotInfo.h"
#include "src/subsystems/Drive.h"
#include "src/subsystems/HatchIntake.h"
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
     * @param greylight The GreyLight system.
     * @param limelightCargo The Limelight for the cargo.
     * @param limelightHatch The Limelight for the hatch.
     */
    Teleop(ObservablePoofsJoystick *driver, ObservableXboxJoystick *codriver,
           Drive *drive, HatchIntake *hatchintake, Limelight *limelightCargo,
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

    static constexpr Color END_GAME_RED = {
        255, 0, 0}; /**< Display red during end game. */
    static constexpr Color NO_COLOR = {0, 0, 0}; /**< Turn off the LED strip. */

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

    HatchIntake *m_hatchIntake;

    LightPattern::Flash *m_endGameSignal;
    bool m_endGameSignalSent;

    Limelight *m_limelightCargo;
    Limelight *m_limelightHatch;

    u_int32_t m_limelightCargoTimer;
    u_int32_t m_limelightHatchTimer;
};
}
