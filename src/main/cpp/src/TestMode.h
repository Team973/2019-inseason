#pragma once

#include "frc/WPILib.h"
#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"
#include "lib/pixelprocessors/Flash.h"
#include "lib/util/Util.h"
#include "src/info/RobotInfo.h"
#include "src/subsystems/Drive.h"
#include "src/subsystems/Elevator.h"
#include "src/subsystems/HatchIntake.h"
#include "src/subsystems/Stinger.h"
#include <iostream>

using namespace frc;

namespace frc973 {

/**
 * Controls the test mode.
 */
class Test {
public:
    /**
     * Constuct a test mode.
     * @param driver The driver's joystick.
     * @param codriver The co-driver's joystick.
     * @param drive The drive subsystem.
     * @param greylight The GreyLight system.
     */
    Test(ObservablePoofsJoystick *driver, ObservableXboxJoystick *codriver,
         Drive *drive, Elevator *elevator, HatchIntake *hatchIntake,
         Stinger *stinger);
    virtual ~Test();

    /**
     * Start of test.
     */
    void TestInit();

    /**
     * Loop of test.
     */
    void TestPeriodic();

    /**
     * Stop of test.
     */
    void TestStop();

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
    enum class DriveMode
    {
        Openloop
    };

    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;

    Drive *m_drive;
    DriveMode m_driveMode;

    Elevator *m_elevator;
    HatchIntake *m_hatchIntake;

    Stinger *m_stinger;
    LightPattern::Flash *m_endGameSignal;
    bool m_endGameSignalSent;
};
}
