/*
 * PIDDriveController.h
 *
 *  Created on: Nov 5, 2015
 *      Author: Andrew
 */

#pragma once

#include "lib/bases/DriveBase.h"
#include "lib/helpers/PID.h"

using namespace frc;

namespace frc973 {

/**
 * PID Drive controller.
 */
class PIDDriveController : public DriveController {
public:
    /**
     * Construct a PID Drive controller.
     */
    PIDDriveController();
    virtual ~PIDDriveController();

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
        return m_onTarget;
    }

    /**
     * Set the target position/heading relative to absolute world.
     * @param dist Distance to travel.
     * @param heading Heading when moving.
     * @param relativity Point relative to new setpoint.
     * @param state The state provider for handling incoming messages.
     */
    PIDDriveController *SetTarget(double dist, double heading,
                                  DriveBase::RelativeTo relativity,
                                  DriveStateProvider *state);

    /**
     * Set the maximum velocities.
     * @param new_vmax_ips The new maximum velocity in inches/second.
     * @param new_avmax_dps The new maximum angular velocity in degrees/second.
     */
    PIDDriveController *SetVMax(double new_vmax_ips, double new_avmax_dps) {
        m_vmax = new_vmax_ips;
        m_avmax = new_avmax_dps;
        m_drivePID->SetBounds(-m_vmax, m_vmax);
        m_turnPID->SetBounds(-m_avmax, m_avmax);
        return this;
    }

    /**
     * Scale the pseed down by newCap
     * @param newCap The new cap (1.0 is max).
     */
    void SetCap(double newCap) {
        m_speedCap = Util::bound(newCap, 0.0, 1.0);
    }

    /**
     * Set the tolerance for distance exiting
     * @param dist Distance tolerance.
     * @param rate Rate tolerance.
     */
    PIDDriveController *SetDistTolerance(double dist = 2.0, double rate = 2.0) {
        m_distTolerance = dist;
        m_distRateTolerance = rate;
        return this;
    }

    /**
     * Set the tolerance for distance exiting
     * @param angle Angle tolerance.
     * @param rate Angle-Rate tolerance.
     */
    PIDDriveController *SetAngleTolerance(double angle = 2.0,
                                          double rate = 2.0) {
        m_angleTolerance = angle;
        m_angleRateTolerance = rate;
        return this;
    }

    /**
     * Default variables.
     */
    void Zero() {
        m_prevDist = 0.0;
        m_prevAngle = 0.0;
        m_targetDist = 0.0;
        m_targetAngle = 0.0;
        m_onTarget = false;
    }
    /**
     * Return the distance error.
     * @return The distance error.
     */
    double GetDistError() {
        return m_targetDist - m_prevDist;
    }

    /**
     * Start the drive controller.
     * @param out The signal receiver for handling outgoing messages.
     */
    void Start(DriveControlSignalReceiver *out) override {
        printf("Turning on PID Mode\n");
    }

    /**
     * Stop the drive controller.
     * @param out The signal receiver for handling outgoing messages.
     */
    void Stop(DriveControlSignalReceiver *out) override {
        printf("Turning off PID Mode\n");
    }

private:
    double m_prevDist;
    double m_prevAngle;

    double m_targetDist;
    double m_targetAngle;

    bool m_onTarget;

    PID *m_drivePID;
    PID *m_turnPID;

    double m_speedCap;
    double m_vmax;
    double m_avmax;

    double m_distTolerance;
    double m_distRateTolerance;
    double m_angleTolerance;
    double m_angleRateTolerance;

    /**
     * In in/sec
     */
    static constexpr double DEFAULT_DIST_TOLERANCE = 3.0;
    static constexpr double DEFAULT_DIST_RATE_TOLERANCE = 5.0;

    /**
     * In deg/sec
     */
    static constexpr double DEFAULT_ANGLE_TOLERANCE = 5.0;
    static constexpr double DEFAULT_ANGLE_RATE_TOLERANCE = 5.0;
};
}
