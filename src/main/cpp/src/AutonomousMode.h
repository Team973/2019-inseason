#pragma once

#include "frc/WPILib.h"
#include <iostream>
#include "src/info/RobotInfo.h"
#include "lib/bases/AutoRoutineBase.h"
#include "src/auto/NoAuto.h"
#include "src/auto/ForwardAuto.h"
#include "lib/util/WrapDash.h"
#include "src/DisabledMode.h"
#include "src/Robot.h"
#include "src/subsystems/Drive.h"
#include "lib/helpers/GreyLight.h"
#include "lib/pixelprocessors/AutoIndicator.h"

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
     */
    Autonomous(Disabled *disabled, Drive *drive, ADXRS450_Gyro *gyro);
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
};
}
