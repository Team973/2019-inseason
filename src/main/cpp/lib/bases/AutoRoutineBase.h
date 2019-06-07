#pragma once

#include <iostream>

namespace frc973 {

/**
 * Auto Routine base.
 */
class AutoRoutineBase {
public:
    /**
     * Construct an AutoRoutineBase.
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
