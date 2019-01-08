#pragma once

#include "frc/WPILib.h"
#include <iostream>

using namespace frc;

namespace frc973 {

/**
 * Auto Routine base.
 */
class AutoRoutineBase {
public:
    /**
     * Construct a Auto Routine base.
     */
    AutoRoutineBase();
    virtual ~AutoRoutineBase();

    /**
     * Called every robot cycle, runs state machine. Overwritten by child class.
     */
    virtual void Execute();

    /**
     * Resets the auto to the beginning.
     */
    virtual void Reset();

protected:
    /**
     * Current step of the auto routine.
     */
    int m_autoState;
};
}
