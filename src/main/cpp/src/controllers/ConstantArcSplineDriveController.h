/**
 * ConstantArcSplineDriveController.h
 * Created On: Summer of 2017
 * Author: Kyle
 *
 **/

#pragma once

#include "lib/bases/DriveBase.h"
#include "lib/helpers/PID.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/profiles/MotionProfile.h"
#include "ctre/Phoenix.h"
#include <stdio.h>

using namespace frc;

namespace frc973 {

/**
 * Constant Arc Spline Drive contoller.
 */
class ConstantArcSplineDriveController : public DriveController {
public:
    /**
     * Construct a Constant Arc Spline Drive controller.
     * @param state The state provider for handling incoming messages.
     * @param logger Logger for drive controller.
     */
    ConstantArcSplineDriveController(DriveStateProvider *state,
                                     LogSpreadsheet *logger);
    virtual ~ConstantArcSplineDriveController();

    /**
     * Sets target for robot to reach.
     * @param relativeTo Point relative to the new setpoint.
     * @param dist Distance to travel.
     * @param angle Angle to turn.
     */
    void SetTarget(DriveBase::RelativeTo relativeTo, double dist, double angle);

    /**
     * Sets the maximum velocity/acceleration.
     * @param max_vel Maximum velocity.
     * @param max_acc Maximum acceleration.
     */
    ConstantArcSplineDriveController *SetMaxVelAccel(double max_vel,
                                                     double max_acc);

    /**
     * Sets the start/end velocity.
     * @param start_vel Start velocity.
     * @param end_vel End velocity.
     */
    ConstantArcSplineDriveController *SetStartEndVel(double start_vel,
                                                     double end_vel);

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
    void Start(DriveControlSignalReceiver *out) override;

    /**
     * Stop the drive controller.
     * @param out The signal receiver for handling outgoing messages.
     */
    void Stop(DriveControlSignalReceiver *out) override;

    /**
     * Return distance from start.
     * @return Distance from start.
     */
    double DistFromStart() const;

    /**
     * Return angle from start.
     * @return Angle from start.
     */
    double AngleFromStart() const;

private:
    DriveStateProvider *m_state;
    double m_dist, m_angle;
    double m_dist_offset, m_angle_offset, m_time_offset;
    double m_max_vel, m_max_acc;
    double m_start_vel, m_end_vel;

    /* pid for linear {pos,vel} */
    PID m_l_pos_pid, m_l_vel_pid;

    /* pid for angular {pos,vel} */
    PID m_a_pos_pid, m_a_vel_pid;

    bool m_done;
    bool m_needSetControlMode;

    static constexpr double MAX_VELOCITY = 130;       // in/sec
    static constexpr double MAX_ACCELERATION = 70.0;  // in/sec^2

    LogCell *m_l_pos_setpt_log;
    LogCell *m_l_pos_real_log;
    LogCell *m_l_vel_setpt_log;
    LogCell *m_l_vel_real_log;
    LogCell *m_a_pos_setpt_log;
    LogCell *m_a_pos_real_log;
    LogCell *m_a_vel_setpt_log;
    LogCell *m_max_vel_log;
    LogCell *m_max_acc_log;
    LogCell *m_dist_endgoal_log;
    LogCell *m_angle_endgoal_log;
    LogCell *m_left_output;
    LogCell *m_right_output;
};
}
