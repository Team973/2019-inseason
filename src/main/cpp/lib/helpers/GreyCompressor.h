/*
 * Compressor.h
 *
 *  Created on: Nov 3, 2015
 *      Author: Andrew
 */

#pragma once

#include "lib/managers/CoopTask.h"
#include "lib/managers/TaskMgr.h"

namespace frc973 {

/**
 * GreyCompressor is super simple CoopTask that checks the given pressure
 * switch and turns on or off the given compressor relay accordingly.
 *
 * Can also be explicitly enabled or disabled.
 */
class GreyCompressor : public CoopTask {
public:
    /**
     * GreyCompressor creates a new compressor object. The name Compressor
     * conflicts with wpilib's compressor object which requires us to run the
     * compressor off the pcm.
     * @param pressureSwitch The pressure switch that it will read from.
     * @param compressor The relay that the compressor is attached to.
     * @param scheduler The task manager that will continually call this.
     */
    explicit GreyCompressor(DigitalInput *pressureSwitch, Relay *compressor,
                            TaskMgr *scheduler);
    virtual ~GreyCompressor();

    /**
     * Enable re-enables compressor running if it was disabled previously.
     */
    void Enable();

    /**
     * Disable disables compressor. Even if the pressure switch reads low, we
     * will not run the compressor until Enable is called.
     */
    void Disable();

    /**
     * Periodic task called by TaskMgr. Checks pressure switch and turns on or
     * off compressor relay accordingly.
     * @param mode The current RobotMode.
     */
    void TaskPeriodic(RobotMode mode);

private:
    bool m_enabled;
    DigitalInput *m_airPressureSwitch;
    Relay *m_compressor;

    TaskMgr *m_scheduler;
};
}
