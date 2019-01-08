#pragma once

#include "frc/WPILib.h"
#include <iostream>
#include "lib/bases/AutoRoutineBase.h"

using namespace frc;
namespace frc973 {
/**
 * No auto.
 */
class NoAuto : public AutoRoutineBase {
public:
    /**
     * Construct no auto.
     */
    NoAuto();
    virtual ~NoAuto();

    /**
     * Resets the auto to the beginning.
     */
    void Reset() override;
};
}
