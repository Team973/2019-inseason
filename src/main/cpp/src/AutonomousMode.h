#pragma once

#include "frc/WPILib.h"
#include <iostream>
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
    Autonomous(Disabled *disabled, Drive *drive, Elevator *elevator,
               ADXRS450_Gyro *gyro, PresetHandlerDispatcher *presetDispatcher);
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

    friend class PresetHandlerDispatcher;

private:
    NoAuto *m_noAuto;
    ForwardAuto *m_forwardAuto;
    GameMode m_gameMode;

    Disabled *m_disabled;

    AutoRoutineBase *m_routine;

    Drive *m_drive;
    Elevator *m_elevator;
    PresetHandlerDispatcher *m_presetDispatcher;
    ADXRS450_Gyro *m_gyro;
};
}
