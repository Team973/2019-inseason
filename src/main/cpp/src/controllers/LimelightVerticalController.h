/*
 *  LimelightVerticalController.h
 *  Created: 4 December 2018
 */

#pragma once

#include "lib/helpers/GreyTalon.h"
#include "lib/helpers/PID.h"
#include "lib/sensors/Limelight.h"
#include "lib/util/Util.h"

namespace frc973 {

/**
 * Limelight Vertical Controller
 */
class LimelightVerticalController {
public:
    /**
     * Constructs a Limelight vertical controller.
     * @param limelight The limelight.
     * @param motor The motor for the vertical subsystem.
     */
    LimelightVerticalController(Limelight *limelight, GreyTalonSRX *motor);
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
        200.0; /**< Velocity multiplier in native units per degree */

private:
    bool m_onTarget;
    double m_setpoint;

    Limelight *m_limelight;
    GreyTalonSRX *m_motor;
};
}
