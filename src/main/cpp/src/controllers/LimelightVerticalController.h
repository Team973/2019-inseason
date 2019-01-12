/*
 *  LimelightVerticalController.h
 *  Created: 4 December 2018
 */

#pragma once

#include "stdio.h"
#include "lib/util/Util.h"
#include "lib/sensors/Limelight.h"
#include "ctre/Phoenix.h"

namespace frc973 {

class PID;

class LimelightVerticalController {
public:
    /**
     * Constructs a Limelight vertical controller.
     * @param limelight The limelight.
     * @param motor The motor for the vertical subsystem.
     */
    LimelightVerticalController(Limelight *limelight, TalonSRX *motor);
    virtual ~LimelightVerticalController();

    /**
     * Start the controller.
     */
    void Start();

    /**
     * Calculate motor output given the most recent limelight updates.
     */
    void CalcOutput();

    /**
     * Checks with the controller to see if setpoint is less than 5.0.
     * @return Whether the controller is at/near setpoint.
     */
    bool OnTarget() {
        return m_onTarget;
    };

    static constexpr double VELOCITY_MULTIPLIER =
        200.0;  // in native units per degree

private:
    bool m_onTarget;
    double m_setpoint;

    Limelight *m_limelight;
    TalonSRX *m_motor;
};
}
