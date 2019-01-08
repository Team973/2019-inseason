#pragma once

#include "frc/WPILib.h"
#include <iostream>
#include "lib/bases/AutoRoutineBase.h"
#include "src/subsystems/Drive.h"

using namespace frc;
namespace frc973 {

/**
 * Forward auto.
 */
class ForwardAuto : public AutoRoutineBase {
public:
    /**
     * Construct a Forward auto.
     * @param drive Drive subsystem.
     */
    ForwardAuto(Drive *drive);
    virtual ~ForwardAuto();

    /**
     * Called every robot cycle, runs state machine.
     */
    void Execute() override;

    /**
     * Resets the auto to the beginning.
     */
    void Reset() override;

private:
    Drive *m_drive;
};
}
