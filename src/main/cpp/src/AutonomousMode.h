#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"

#include "src/info/RobotInfo.h"
#include "src/TeleopMode.h"

namespace frc973 {

/**
 * Controls the Autonomous mode.
 */
class Autonomous {
public:
    /**
     * Constuct an Autonomous mode.
     * @param driverJoystick The driver's ObservablePoofsJoystick.
     * @param operatorJoystick The operator's ObservableXboxJoystick.
     * @param tuningJoystick The tuning ObservableDualActionJoystick.
     * @param Teleop The Teleop mode.
     * @param gyro The PigeonIMU.
     * @param drive The Drive subsystem.
     * @param cargoIntake The CargoIntake subsystem.
     * @param hatchIntake The HatchIntake subsystem.
     * @param elevator The Elevator subsystem.
     */
    Autonomous(ObservablePoofsJoystick *driverJoystick,
               ObservableXboxJoystick *operatorJoystick,
               ObservableDualActionJoystick *tuningJoystick, Teleop *Teleop,
               PigeonIMU *gyro, Drive *drive, CargoIntake *cargoIntake,
               HatchIntake *hatchIntake, Elevator *elevator,
               Limelight *limelightHatch);
    virtual ~Autonomous();

    /**
     * Start of Autonomous.
     */
    void AutonomousInit();

    /**
     * Loop of Autonomous.
     */
    void AutonomousPeriodic();

    /**
     * Stop of Autonomous.
     */
    void AutonomousStop();

    /**
     * Button handler for the Autonomous mode.
     * @param port The port the joystick is connected to.
     * @param button The button.
     * @param pressedP The button's new status.
     */
    void HandleDualActionJoystick(uint32_t port, uint32_t button,
                                  bool pressedP);

    /**
     * Button handler for the Autonomous mode.
     * @param port The port the joystick is connected to.
     * @param button The button.
     * @param pressedP The button's new status.
     */
    void HandlePoofsJoystick(uint32_t port, uint32_t button, bool pressedP);

    /**
     * Button handler for the Autonomous mode.
     * @param port The port the joystick is connected to.
     * @param button The button.
     * @param pressedP The button's new status.
     */
    void HandleXboxJoystick(uint32_t port, uint32_t button, bool pressedP);

    /**
     * Defines AutoStates.
     */
    enum class AutoState
    {
        ForwardAuto,         /**< Basic Forward Auto. */
        SingleHatchAuto,     /**< Place a single hatch on the cargo ship. */
        TwoRocket,           /**< Two rocket objects. */
        TwoCargoShip,        /**< Two cargo objects. */
        CargoShipThenRocket, /**< One on both the cargo and rocket. */
        Manual,              /**< Manual driver control. */
        NoAuto,              /**< No autonomous. */
        DoubleHatchAuto,     /**< Place a single hatch plus... */
        TurnAuto,            /**< Turn 90 degrees. */
        CargoToHuman,
    };

    /**
     * Gets the current AutoState.
     * @return The current AuutoState.
     */
    AutoState GetAutoState() const;

    /**
     * Sets the AutoState.
     * @param autoState The new AutoState.
     */
    void SetAutoState(AutoState autoState);

    /**
     * Defines AutoStateStartPositions.
     */
    enum class AutoStateStartPosition
    {
        LeftHabLevel2, /**< Left Level 2 Auto State. */
        CenterHab,     /**< Center Auto State. */
        RightHabLevel2 /**< Right Level 2 Auto State. */
    };

    double timer;

private:
    void ForwardAuto();
    void SingleHatchAuto();
    void TwoRocketAuto();
    void TwoRocketAutoFront();
    void TwoRocketAutoBack();
    void TwoCargoShipAuto();
    void CargoShipThenRocketAuto(const bool doCargoOnly = false);
    void CargoToHumanPlayer();
    void NoAuto();
    void DoubleHatchAuto();
    void TurnAuto();

    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;
    ObservableDualActionJoystick *m_tuningJoystick;

    Teleop *m_teleop;
    AutoState m_autoState;
    AutoStateStartPosition m_autoStateStartPosition;
    double m_autoTimer;
    double m_direction;
    int m_autoStep;
    // ADXRS450_Gyro *m_gyro;
    PigeonIMU *m_gyro;

    Drive *m_drive;
    CargoIntake *m_cargoIntake;
    HatchIntake *m_hatchIntake;
    Elevator *m_elevator;

    Limelight *m_limelightHatch;
};
}
