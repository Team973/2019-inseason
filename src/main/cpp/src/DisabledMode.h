#pragma once

#include "frc/WPILib.h"
#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/GreyLight.h"
#include "src/subsystems/Elevator.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"
#include "lib/pixelprocessors/SolidColor.h"
#include "lib/sensors/Limelight.h"
#include "lib/util/WrapDash.h"
#include "src/AutonomousMode.h"
#include "src/Robot.h"
#include "lib/bases/AutoRoutineBase.h"
#include "src/info/RobotInfo.h"
#include <iostream>

using namespace frc;
using namespace cs;

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
     */
    Disabled(ObservablePoofsJoystick *driver, ObservableXboxJoystick *codriver,
             Elevator *elevator, Limelight *limelightCargo,
             Limelight *limelightHatch);
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

    Limelight *m_limelightCargo;
    Limelight *m_limelightHatch;
};
}
