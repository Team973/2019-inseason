#pragma once

#include "lib/helpers/DualActionJoystickHelper.h"

#include "src/subsystems/CargoIntake.h"
#include "src/subsystems/Drive.h"
#include "src/subsystems/CargoIntake.h"
#include "src/subsystems/Elevator.h"
#include "src/subsystems/HatchIntake.h"
#include "src/subsystems/Stinger.h"

namespace frc973 {
/**
 * Handles preset selection.
 */
class PresetHandlerDispatcher;

/**
 * Controls the test mode.
 */
class Test {
public:
    /**
     * Constuct a test mode.
     * @param driver The driver's joystick.
     * @param codriver The co-driver's joystick.
     * @param testStick The test joystick.
     * @param drive The drive subsystem.
     * @param elevator The elevator subsystem.
     * @param hatchIntake The hatch intake subsystem.
     * @param cargoIntake The cargo intake subsystem
     * @param stinger The stinger subsystem.
     * @param limelightHatch The hatch limelight.
     */
    Test(ObservablePoofsJoystick *driver, ObservableXboxJoystick *codriver,
         ObservableDualActionJoystick *testStick, Drive *drive,
         Elevator *elevator, HatchIntake *hatchIntake, CargoIntake *cargoIntake,
         Stinger *stinger, Limelight *limelightHatch);
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

    friend class PresetHandlerDispatcher;

private:
    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;
    ObservableDualActionJoystick *m_testJoystick;

    Drive *m_drive;
    enum class DriveMode
    {
        Openloop,
        LimelightHatch,
        LimelightTrig,
        AssistedCheesy,
        Cheesy,
        PIDDrive,
        PIDTurn
    };
    DriveMode m_driveMode;

    HatchIntake *m_hatchIntake;
    Elevator *m_elevator;
    CargoIntake *m_cargoIntake;
    Stinger *m_stinger;

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
