#pragma once

#include "frc/WPILib.h"
#include "src/info/RobotInfo.h"
#include "src/TeleopMode.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"
#include "lib/helpers/DualActionJoystickHelper.h"

using namespace frc;

namespace frc973 {

/**
 * Controls the autonomous mode.
 */
class Autonomous {
public:
    /**
     * Constuct an autonomous mode.
     * @param driver The driver controller.
     * @param codriver The codriver controller.
     * @param testJoystick The test controller.
     * @param teleop The teleop mode.
     * @param gyro The gyro.
     * @param drive The drive subsystem.
     * @param cargoIntake The cargo intake subsystem.
     * @param hatchIntake The hatch intake subsystem.
     * @param elevator The elevator subsystem.
     */
    Autonomous(ObservablePoofsJoystick *driver,
               ObservableXboxJoystick *codriver,
               ObservableDualActionJoystick *testJoystick, Teleop *teleop,
               ADXRS450_Gyro *gyro, Drive *drive, CargoIntake *cargoIntake,
               HatchIntake *hatchIntake, Elevator *elevator);
    virtual ~Autonomous();

    /**
     * Start of autonomous.
     */
    void AutonomousInit();

    /**
     * Loop of autonomous.
     */
    void AutonomousPeriodic();

    /**
     * Stop of autonomous.
     */
    void AutonomousStop();

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

    /*
     * Defines AutoStates.
     */
    enum class AutoState
    {
        TwoRocket,
        TwoCargoShip,
        CargoShipThenRocket,
        Manual
    };

    /*
     * Gets AutoState
     */
    AutoState GetAutoState() const;

    /*
     * Sets AutoState
     */
    void SetAutoState(AutoState autoState);

    /*
     * Defines AutoStateStartPositions
     */

    enum class AutoStateStartPosition
    {
        LeftHabLevel2,
        CenterHab,
        RightHabLevel2
    };

private:
    void TwoRocketAuto();
    void TwoRocketAutoFront();
    void TwoRocketAutoBack();
    void TwoCargoShipAuto();
    void CargoShipThenRocketAuto(const bool doCargoOnly = false);

    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;
    ObservableDualActionJoystick *m_testJoystick;

    Teleop *m_teleop;
    AutoState m_autoState;
    AutoStateStartPosition m_autoStateStartPosition;
    double m_autoTimer;
    double m_direction;
    int m_autoStep;
    ADXRS450_Gyro *m_gyro;

    Drive *m_drive;
    CargoIntake *m_cargoIntake;
    HatchIntake *m_hatchIntake;
    Elevator *m_elevator;
};
}
