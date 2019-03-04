#include "src/controllers/SplineDriveController.h"
#include "src/info/RobotInfo.h"
#include "lib/util/Util.h"
#include "lib/util/WrapDash.h"
#include <cmath>

namespace frc973 {

using namespace Constants;
using namespace trajectories;

static constexpr double POSITION_KP = 1.1;
static constexpr double POSITION_KI = 0.0;
static constexpr double POSITION_KD = 0.0;

static constexpr double VELOCITY_KP = 1.3;
static constexpr double VELOCITY_KI = 0.0;
static constexpr double VELOCITY_KD = 0.0;

static constexpr double ANGULAR_POSITION_KP = 4.6;
static constexpr double ANGULAR_POSITION_KI = 0.0;
static constexpr double ANGULAR_POSITION_KD = 0.0;

static constexpr double ANGULAR_RATE_KP = 0.0;
static constexpr double ANGULAR_RATE_KI = 0.0;
static constexpr double ANGULAR_RATE_KD = 0.0;

static constexpr double ACCEL_FF = 0.2;
static constexpr double ANGLE_RATE_FF = 0.0;
static constexpr double ANGLE_ACCEL_FF = 0.15;

SplineDriveController::SplineDriveController(DriveStateProvider *state,
                                             LogSpreadsheet *logger)
        : DriveController()
        , m_state(state)
        , m_trajectory(nullptr)
        , m_left_dist_offset(0.0)
        , m_right_dist_offset(0.0)
        , m_angle_offset(0.0)
        , m_time_offset(0.0)
        , m_done(false)
        , m_l_pos_pid(POSITION_KP, POSITION_KI, VELOCITY_KD)
        , m_l_vel_pid(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
        , m_r_pos_pid(POSITION_KP, POSITION_KI, POSITION_KD)
        , m_r_vel_pid(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD)
        , m_a_pos_pid(ANGULAR_POSITION_KP, ANGULAR_POSITION_KI,
                      ANGULAR_POSITION_KD)
        , m_a_rate_pid(ANGULAR_RATE_KP, ANGULAR_RATE_KI, ANGULAR_RATE_KD) {
    m_l_pos_pid.SetBounds(-100, 100);
    m_l_vel_pid.SetBounds(-100, 100);
    m_r_pos_pid.SetBounds(-100, 100);
    m_r_vel_pid.SetBounds(-100, 100);
    m_a_pos_pid.SetBounds(-100, 100);
    m_a_rate_pid.SetBounds(-100, 100);
}

SplineDriveController::~SplineDriveController() {
    ;
}

void SplineDriveController::SetTarget(TrajectoryDescription *trajectory,
                                      DriveBase::RelativeTo relativity) {
    m_time_offset = GetSecTime();

    m_left_dist_offset = m_state->GetLeftDist();
    m_right_dist_offset = m_state->GetRightDist();

    m_trajectory = trajectory;

    switch (relativity) {
        case DriveBase::RelativeTo::Now:
            m_angle_offset = m_state->GetAngle();
            break;
        case DriveBase::RelativeTo::SetPoint:
            // m_angle_offset = trajectories::GetHeadingDegrees(m_trajectory,
            // 0.0);
            break;
        case DriveBase::RelativeTo::Absolute:
            break;
    }
}

void SplineDriveController::CalcDriveOutput(DriveStateProvider *state,
                                            DriveControlSignalReceiver *out) {
    double time = GetSecTime() - m_time_offset;

    double leftDist = trajectories::GetLeftDist(m_trajectory, time);
    double rightDist = trajectories::GetRightDist(m_trajectory, time);
    double leftVel = trajectories::GetLeftDriveVelocity(m_trajectory, time);
    double rightVel = trajectories::GetRightDriveVelocity(m_trajectory, time);
    double heading = trajectories::GetHeadingDegrees(m_trajectory, time);
    double angularRate =
        trajectories::GetAngularRateDegrees(m_trajectory, time);

    m_l_pos_pid.SetTarget(leftDist);
    m_r_pos_pid.SetTarget(rightDist);
    m_l_vel_pid.SetTarget(leftVel);
    m_r_vel_pid.SetTarget(rightVel);
    m_a_pos_pid.SetTarget(heading);
    m_a_rate_pid.SetTarget(angularRate);
    double angle_error = Util::CalcAngleError(heading, AngleFromStart());

    /* vel feed forward for linear term */
    double right_l_vel_ff =
        trajectories::GetRightDriveVelocity(m_trajectory, time);
    double left_l_vel_ff =
        trajectories::GetLeftDriveVelocity(m_trajectory, time);
    double leftAccel_ff =
        ACCEL_FF * trajectories::GetLeftAcceleration(m_trajectory, time);
    double rightAccel_ff =
        ACCEL_FF * trajectories::GetRightAcceleration(m_trajectory, time);
    double angleAccel_ff =
        ANGLE_ACCEL_FF *
        trajectories::GetAngularAcceleration(m_trajectory, time);
    double angleRate_ff =
        ANGLE_RATE_FF * trajectories::GetAngularRateDegrees(m_trajectory, time);

    /* correction terms for error in {linear,angular} {position,velocioty */
    double left_linear_dist_term = m_l_pos_pid.CalcOutput(LeftDistFromStart());
    double left_linear_vel_term = m_l_vel_pid.CalcOutput(state->GetLeftRate());
    double right_linear_dist_term =
        m_r_pos_pid.CalcOutput(RightDistFromStart());
    double right_linear_vel_term =
        m_r_vel_pid.CalcOutput(state->GetRightRate());
    double angular_dist_term = m_a_pos_pid.CalcOutputWithError(angle_error);
    double angular_rate_term = m_a_rate_pid.CalcOutput(state->GetAngularRate());

    /* right side receives positive angle correction */
    double right_output = right_l_vel_ff + right_linear_dist_term +
                          right_linear_vel_term + angular_dist_term +
                          rightAccel_ff + angleAccel_ff + angular_rate_term +
                          angleRate_ff;
    /* left side receives negative angle correction */
    double left_output = left_l_vel_ff + left_linear_dist_term +
                         left_linear_vel_term - angular_dist_term +
                         leftAccel_ff - angleAccel_ff - angular_rate_term -
                         angleRate_ff;

    out->SetDriveOutputIPS(left_output, right_output);

    if (time < trajectories::GetLength(m_trajectory) *
                   m_trajectory->left_trajectory->dt) {
        m_done = false;
    }
    else {
        m_done = true;
    }
}

double SplineDriveController::GetSplinePercentComplete() const {
    return trajectories::GetPercentComplete(m_trajectory,
                                            GetSecTime() - m_time_offset);
}

double SplineDriveController::LeftDistFromStart() const {
    return m_state->GetLeftDist() - m_left_dist_offset;
}

double SplineDriveController::RightDistFromStart() const {
    return m_state->GetRightDist() - m_right_dist_offset;
}

double SplineDriveController::AngleFromStart() const {
    return m_state->GetAngle() - m_angle_offset;
}
}
