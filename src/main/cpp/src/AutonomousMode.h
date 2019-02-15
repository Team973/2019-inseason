#pragma once

#include "frc/WPILib.h"
#include "src/info/RobotInfo.h"
#include "lib/bases/AutoRoutineBase.h"
#include "src/auto/NoAuto.h"
#include "src/auto/ForwardAuto.h"
#include "lib/util/WrapDash.h"
#include "src/DisabledMode.h"
#include "src/subsystems/Elevator.h"
#include "src/Robot.h"
#include "src/subsystems/Drive.h"
#include "lib/pixelprocessors/AutoIndicator.h"
#include "src/GameMode.h"

using namespace frc;

namespace frc973 {
class Disabled;
class PresetHandlerDispatcher;

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
               ObservableXboxJoystick *codriver, Disabled *disabled,
               Drive *drive, Elevator *elevator, HatchIntake *hatchIntake,
               CargoIntake *cargoIntake, ADXRS450_Gyro *gyro,
               PresetHandlerDispatcher *presetDispatcher);
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

    friend class PresetHandlerDispatcher;

private:
    NoAuto *m_noAuto;
    ForwardAuto *m_forwardAuto;
    GameMode m_gameMode;

    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;

    Disabled *m_disabled;

    AutoRoutineBase *m_routine;

    Drive *m_drive;
    enum class DriveMode
    {
        Openloop,
        LimelightCargo,
        LimelightHatch,
        AssistedCheesyHatch,
        AssistedCheesyCargo,
        Cheesy
    };
    DriveMode m_driveMode;
    Elevator *m_elevator;
    CargoIntake *m_cargoIntake;
    HatchIntake *m_hatchIntake;
    PresetHandlerDispatcher *m_presetDispatcher;
    ADXRS450_Gyro *m_gyro;
};
}
