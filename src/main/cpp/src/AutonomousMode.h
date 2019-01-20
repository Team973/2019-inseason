#pragma once

#include "frc/WPILib.h"
#include "src/info/RobotInfo.h"
#include "lib/bases/AutoRoutineBase.h"
#include "src/auto/NoAuto.h"
#include "src/auto/ForwardAuto.h"
#include "lib/util/WrapDash.h"
#include "src/DisabledMode.h"
#include "src/Robot.h"
#include "src/subsystems/Drive.h"
#include "lib/pixelprocessors/AutoIndicator.h"
#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"
#include "lib/pixelprocessors/Flash.h"
#include "ctre/Phoenix.h"
#include "lib/helpers/GreyTalon.h"
#include "lib/managers/CoopTask.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/sensors/Limelight.h"
#include "src/controllers/LimelightVerticalController.h"
#include "lib/util/Util.h"
#include <iostream>

using namespace frc;

namespace frc973 {
class Disabled;

/**
 * Controls the autonomous mode.
 */
class Autonomous {
public:
    /**
     * Constuct an autonomous mode.
     * @param disabled The disabled mode.
     * @param drive The drive subsystem.
     * @param gyro The gyro.
     * @param greylight The GreyLight system.
     */
    Autonomous(Disabled *disabled, Drive *drive, ADXRS450_Gyro *gyro,
               ObservablePoofsJoystick *driver,
               ObservableXboxJoystick *codriver);
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

private:
    NoAuto *m_noAuto;
    ForwardAuto *m_forwardAuto;

    Disabled *m_disabled;

    AutoRoutineBase *m_routine;

    Drive *m_drive;
    ADXRS450_Gyro *m_gyro;

    LightPattern::AutoIndicator *m_autoSignal;
    LightPattern::Flash *m_endGameSignal;
    bool m_endGameSignalSent;

    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;

    /*
     * Defines DriveModes
     * Openloop: Arcade
     * Cheesy: Turn is a function of drive
     */

    enum class DriveMode
    {
        Openloop,
        Cheesy
    };
    DriveMode m_driveMode;
};
}
