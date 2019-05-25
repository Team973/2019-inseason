#pragma once

#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"
#include "lib/sensors/Limelight.h"
#include "lib/util/WrapDash.h"

#include "src/subsystems/Drive.h"
#include "src/subsystems/Elevator.h"
#include "src/AutonomousMode.h"
#include "src/TeleopMode.h"

namespace frc973 {

/**
 * Controls the disabled mode.
 */
class Disabled {
public:
    /**
     * Constuct a disabled mode.
     * @param driver The driver's joystick.
     * @param codriver The co-driver's joystick.
     * @param elevator The elevator subsystem.
     * @param cargoIntake The cargo Intake subsystem.
     * @param limelightHatch The hatch limelight.
     * @param autonomous The autonomous mode.
     * @param drive The drive
     * @param teleop The teleop mode
     */
    Disabled(ObservablePoofsJoystick *driver, ObservableXboxJoystick *codriver,
             Elevator *elevator, CargoIntake *cargoIntake, Drive *drive,
             Limelight *limelightHatch, Autonomous *autonomous, Teleop *teleop);
    virtual ~Disabled();

    /**
     * Start of disabled.
     */
    void DisabledInit();

    /**
     * Loop of disabled.
     */
    void DisabledPeriodic();

    /**
     * Stop of disabled.
     */
    void DisabledStop();

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

    Elevator *m_elevator;
    CargoIntake *m_cargoIntake;
    Drive *m_drive;

    Limelight *m_limelightHatch;

    Autonomous *m_autonomous;
    Teleop *m_teleop;
};
}
