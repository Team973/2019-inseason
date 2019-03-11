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
     * @param disabled The disabled mode.
     * @param drive The drive subsystem.
     * @param elevator The elevator subsystem
     * @param gyro The gyro.
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

    enum class AutoState
    {
        TwoRocket,
        TwoCargoShip,
        CargoShipThenRocket,
        Manual
    };

    AutoState GetAutoState() const;
    void SetAutoState(AutoState autoState);

    enum class AutoStateStartPosition
    {
        LeftHabLevel2,
        CenterHab,
        RightHabLevel2
    };

    AutoStateStartPosition GetAutoStateStartPosition() const;
    void SetAutoStateStartPosition(AutoStateStartPosition startPos);

private:
    void TwoRocketAuto();
    void TwoRocketAutoFront();
    void TwoRocketAutoBack();
    void TwoCargoShipAuto();
    void CargoShipThenRocketAuto();

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
