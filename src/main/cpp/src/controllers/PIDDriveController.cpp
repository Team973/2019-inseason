/*
 * PIDDriveController.cpp
 *
 *  Created on: Nov 5, 2015
 *      Author: Andrew
 */

#include "src/controllers/PIDDriveController.h"

namespace frc973 {

// Drive pid takes in error in inches and outputs velocity in inches/sec
static constexpr double DRIVE_PID_KP = 0.0;
static constexpr double DRIVE_PID_KI = 0.0;
static constexpr double DRIVE_PID_KD = 0.0;

// Turn pid takes in error in degrees and outputs velocity in degrees/sec
static constexpr double TURN_PID_KP = 0.0;
static constexpr double TURN_PID_KI = 0.0;
static constexpr double TURN_PID_KD = 0.0;

static constexpr double MAX_LINEAR_SPEED_IPS =
    60.0;  // reduced for testing from: 150.0;          // in in/sec
static constexpr double MAX_ANGULAR_RATE_DEG_PER_SEC =
    180.0;  // reduced for testing from: 360.0;  // in degrees/sec

static constexpr double DRIVE_ARC_IN_PER_DEG =
    DRIVE_WIDTH * Constants::PI / 360.0;

PIDDriveController::PIDDriveController()
        : m_prevDist(0.0)
        , m_prevAngle(0.0)
        , m_targetDist(0.0)
        , m_targetAngle(0.0)
        , m_onTarget(false)
        , m_drivePID(new PID(DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD))
        , m_turnPID(new PID(TURN_PID_KP, TURN_PID_KI, TURN_PID_KD))
        , m_speedCap(1.0)
        , m_vmax(MAX_LINEAR_SPEED_IPS)
        , m_avmax(MAX_ANGULAR_RATE_DEG_PER_SEC)
        , m_distTolerance(DEFAULT_DIST_TOLERANCE)
        , m_distRateTolerance(DEFAULT_DIST_RATE_TOLERANCE)
        , m_angleTolerance(DEFAULT_ANGLE_TOLERANCE)
        , m_angleRateTolerance(DEFAULT_ANGLE_RATE_TOLERANCE) {
    m_drivePID->SetBounds(-m_vmax, m_vmax);
    m_turnPID->SetBounds(-m_avmax, m_avmax);
}

PIDDriveController::~PIDDriveController() {
    delete m_drivePID;
    delete m_turnPID;
}

void PIDDriveController::CalcDriveOutput(DriveStateProvider *state,
                                         DriveControlSignalReceiver *out) {
    m_prevDist = state->GetDist();
    m_prevAngle = state->GetAngle();

    double throttle =
        m_drivePID->CalcOutputWithError(m_targetDist - m_prevDist);
    /*Util::bound(m_drivePID->CalcOutputWithError(m_targetDist - m_prevDist),
    -m_vmax, m_vmax) * m_speedCap;*/
    double turn = m_turnPID->CalcOutputWithError(m_targetAngle - m_prevAngle);
    /*Util::bound(m_turnPID->CalcOutputWithError(m_targetAngle - m_prevAngle),
    -m_avmax, m_avmax) * m_speedCap * DRIVE_ARC_IN_PER_DEG;*/

    out->SetDriveOutputVBus(throttle - turn, throttle + turn);

    if (fabs(m_targetDist - m_prevDist) < m_distTolerance &&
        fabs(state->GetRate()) < m_distRateTolerance &&
        fabs(m_targetAngle - m_prevAngle) < m_angleTolerance &&
        fabs(state->GetAngularRate()) < m_angleRateTolerance) {
        m_onTarget = true;
    }
    else {
        m_onTarget = false;
    }

    /* DBStringPrintf(DBStringPos::DB_LINE3, "th%2.2lf tu%2.2lf", throttle, turn);
    DBStringPrintf(DBStringPos::DB_LINE5, "dt %5.0lf, dc %5.0lf", m_targetDist,
                    m_prevDist);
    DBStringPrintf(DBStringPos::DB_LINE6, "err d %.3lf a %.3lf",
                    m_targetDist - m_prevDist, m_targetAngle - m_prevAngle);
    DBStringPrintf(DB_LINE7, "Drive N: %f, S: %f, E: %f", m_prevDist,
                    m_targetDist, GetDistError());
    DBStringPrintf(DB_LINE8, "Turn N: %f, S: %f, E: %f", m_prevAngle,
                    m_targetAngle, GetAngleError()); */
}
/*
 * dist and angle are RelativeTo current position
 */
PIDDriveController *PIDDriveController::SetTarget(
    double dist, double angle, DriveBase::RelativeTo relativity,
    DriveStateProvider *state) {
    m_drivePID->Reset();
    m_turnPID->Reset();

    switch (relativity) {
        case DriveBase::RelativeTo::Absolute:
            m_targetDist = dist;
            m_targetAngle = angle;
            break;
        case DriveBase::RelativeTo::Now:
            m_targetDist = state->GetDist() + dist;
            m_targetAngle = state->GetAngle() + angle;
            break;
        case DriveBase::RelativeTo::SetPoint:
            m_targetDist = m_targetDist + dist;
            m_targetAngle = m_targetAngle + angle;
            break;
    }

    m_drivePID->SetTarget(m_targetDist);
    m_turnPID->SetTarget(m_targetAngle);

    m_distTolerance = DEFAULT_DIST_TOLERANCE;
    m_distRateTolerance = DEFAULT_DIST_RATE_TOLERANCE;
    m_angleTolerance = DEFAULT_ANGLE_TOLERANCE;
    m_angleRateTolerance = DEFAULT_ANGLE_RATE_TOLERANCE;
    m_vmax = MAX_LINEAR_SPEED_IPS;
    m_avmax = MAX_ANGULAR_RATE_DEG_PER_SEC;
    m_drivePID->SetBounds(-m_vmax, m_vmax);
    m_turnPID->SetBounds(-m_avmax, m_avmax);

    return this;
}
}
