/**
 * SplineDriveController.h
 * Created On: Feb 12, 2018
 * Author: Kyle
 */

#pragma once

#include "lib/bases/DriveBase.h"
#include "lib/helpers/PID.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/trajectories/SplineProfile.h"

namespace frc973 {

using namespace Trajectories;

/**
 * Spline Drive controller.
 */
class SplineDriveController : public DriveController {
public:
    /**
     * Construct a Spline Drive controller.
     * @param state Inputs to the drive controller.
     * @param logger Logger for the drive controller.
     */
    SplineDriveController(DriveStateProvider *state, LogSpreadsheet *logger);
    virtual ~SplineDriveController();

    /**
     * Set the target trajectory.
     * @param trajectory The trajectory.
     * @param relativity The point relative to new setpoint.
     */
    void SetTarget(TrajectoryDescription *trajectory,
                   DriveBase::RelativeTo relativity);

    /**
     * Calculate motor output given the most recent sensor updates.
     * @param state The state provider for handling incoming messages.
     * @param out The signal receiver for handling outgoing messages.
     */
    void CalcDriveOutput(DriveStateProvider *state,
                         DriveControlSignalReceiver *out) override;

    /**
     * Checks with the controller to see if we are on target.
     * @return Whether the controller things are done.
     */
    bool OnTarget() override {
        return m_done;
    }

    /**
     * Start the drive controller.
     * @param out The signal receiver for handling outgoing messages.
     */
    void Start(DriveControlSignalReceiver *out) override {
        printf("Turning on Spline Mode\n");
    }

    /**
     * Stop the drive controller.
     * @param out The signal receiver for handling outgoing messages.
     */
    void Stop(DriveControlSignalReceiver *out) override {
        printf("Turning off Spline Mode\n");
    }

    /**
     * Return percent complete of spline
     * @return Percent of trajectory done.
     */
    double GetSplinePercentComplete() const;

    /**
     * Return left distance from beginning.
     * @return Left distance from beginning.
     */
    double LeftDistFromStart() const;

    /**
     * Return right distance from beginning.
     * @return Right distance from beginning.
     */
    double RightDistFromStart() const;

    /**
     * Return angle from beginning.
     * @return Angle from beginning.
     */
    double AngleFromStart() const;

private:
    DriveStateProvider *m_state;
    TrajectoryDescription *m_trajectory;
    DriveControlSignalReceiver *m_driveOutput;
    double m_left_dist_offset, m_right_dist_offset, m_angle_offset,
        m_time_offset;
    bool m_done;

    /* pid for linear {pos,vel} */
    PID m_l_pos_pid, m_l_vel_pid;
    PID m_r_pos_pid, m_r_vel_pid;

    /* pid for angular pos */
    PID m_a_pos_pid;

    /* pid for angular rate */
    PID m_a_rate_pid;
};
}
