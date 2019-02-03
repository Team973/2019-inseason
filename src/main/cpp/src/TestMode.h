#pragma once

#include "frc/WPILib.h"
#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"
#include "lib/sensors/Limelight.h"
#include "lib/pixelprocessors/Flash.h"
#include "lib/util/WrapDash.h"
#include "lib/util/Util.h"
#include "lib/util/WrapDash.h"
#include "src/info/RobotInfo.h"
#include "src/subsystems/Drive.h"
#include "src/subsystems/CargoIntake.h"
#include "src/subsystems/Elevator.h"
#include "src/subsystems/HatchIntake.h"
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
     */
    Test(ObservablePoofsJoystick *driver, ObservableXboxJoystick *codriver,
         Drive *drive, Elevator *elevator, HatchIntake *hatchIntake,
         CargoIntake *cargoIntake, Limelight *limelightCargo,
         Limelight *limelightHatch);
    virtual ~Test();
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
    Elevator *m_elevator;
    CargoIntake *m_cargoIntake;

    Limelight *m_limelightCargo;
    Limelight *m_limelightHatch;

    enum class Rumble
    {
        on,
        off
    };
    Rumble m_rumble;

    uint32_t m_rumbleTimer;
};
}
