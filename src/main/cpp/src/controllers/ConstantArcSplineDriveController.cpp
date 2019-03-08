#include "src/controllers/ConstantArcSplineDriveController.h"
#include "lib/profiles/MotionProfile.h"
#include "src/info/RobotInfo.h"
#include "lib/util/Util.h"
#include "lib/util/WrapDash.h"

namespace frc973 {

using namespace Constants;

ConstantArcSplineDriveController::ConstantArcSplineDriveController(
    DriveStateProvider *state, LogSpreadsheet *logger)
        : m_state(state)
        , m_dist(0.0)
        , m_angle(0.0)
        , m_dist_offset(0.0)
        , m_angle_offset(0.0)
        , m_time_offset(0.0)
        , m_max_vel(MAX_VELOCITY)
        , m_max_acc(MAX_ACCELERATION)
        , m_start_vel(0.0)
        , m_end_vel(0.0)
        , m_l_pos_pid(1.0, 0.0, 0.0)
        , m_l_vel_pid(0.1, 0.0, 0.0)
        , m_a_pos_pid(1.9, 0.0, 0.0)
        , m_a_vel_pid(0.2, 0.0, 0.0)
        , m_done(false)
        , m_needSetControlMode(false)
        , m_l_pos_setpt_log(new LogCell("s_linear pos incr goal"))
        , m_l_pos_real_log(new LogCell("s_linear pos incr actual"))
        , m_l_vel_setpt_log(new LogCell("s_linear vel incr goal"))
        , m_l_vel_real_log(new LogCell("s_linear vel incr actual"))
        , m_a_pos_setpt_log(new LogCell("s_angular pos incr goal"))
        , m_a_pos_real_log(new LogCell("s_angular pos incr actual"))
        , m_a_vel_setpt_log(new LogCell("s_angular vel incr goal"))
        , m_max_vel_log(new LogCell("ConstantArcSpline max velocity"))
        , m_max_acc_log(new LogCell("ConstantArcSpline max accel"))
        , m_dist_endgoal_log(new LogCell("s_linear pos end goal"))
        , m_angle_endgoal_log(new LogCell("s_angle pos end goal"))
        , m_left_output(new LogCell("s_left output"))
        , m_right_output(new LogCell("s_right output")) {
    m_l_pos_pid.SetBounds(-100, 100);
    m_l_vel_pid.SetBounds(-100, 100);
    m_a_pos_pid.SetBounds(-100, 100);
    m_a_vel_pid.SetBounds(-100, 100);

    if (logger) {
        logger->RegisterCell(m_l_pos_setpt_log);
        logger->RegisterCell(m_l_pos_real_log);
        logger->RegisterCell(m_l_vel_setpt_log);
        logger->RegisterCell(m_l_vel_real_log);
        logger->RegisterCell(m_a_pos_setpt_log);
        logger->RegisterCell(m_a_pos_real_log);
        logger->RegisterCell(m_a_vel_setpt_log);
        logger->RegisterCell(m_max_vel_log);
        logger->RegisterCell(m_max_acc_log);
        logger->RegisterCell(m_dist_endgoal_log);
        logger->RegisterCell(m_angle_endgoal_log);
        logger->RegisterCell(m_left_output);
        logger->RegisterCell(m_right_output);
    }
}

ConstantArcSplineDriveController::~ConstantArcSplineDriveController() {
    ;
}

void ConstantArcSplineDriveController::SetTarget(
    DriveBase::RelativeTo relativeTo, double dist, double angle) {
    m_time_offset = GetSecTime();

    switch (relativeTo) {
        case DriveBase::RelativeTo::Absolute:
            m_dist_offset = 0.0;
            m_angle_offset = 0.0;
            break;
        case DriveBase::RelativeTo::Now:
            m_dist_offset = m_state->GetDist();
            m_angle_offset = m_state->GetAngle();
            break;
        case DriveBase::RelativeTo::SetPoint:
            m_dist_offset += m_dist;
            m_angle_offset += m_angle;
            break;
    }

    m_dist = dist;
    m_angle = angle;

    m_max_vel = MAX_VELOCITY;
    m_max_acc = MAX_ACCELERATION;

    m_start_vel = 0.0;
    m_end_vel = 0.0;
}

ConstantArcSplineDriveController *
ConstantArcSplineDriveController::SetMaxVelAccel(double max_vel,
                                                 double max_acc) {
    m_max_vel = max_vel;
    m_max_acc = max_acc;
    return this;
}

ConstantArcSplineDriveController *
ConstantArcSplineDriveController::SetStartEndVel(double start_vel,
                                                 double end_vel) {
    m_start_vel = start_vel;
    m_end_vel = end_vel;
    return this;
}

void ConstantArcSplineDriveController::CalcDriveOutput(
    DriveStateProvider *state, DriveControlSignalReceiver *out) {
    double time = GetSecTime() - m_time_offset;

    Profiler::NewWaypoint goal;

    if ((Util::square(m_max_vel) / m_max_acc) < fabs(m_dist) ||
        m_max_acc == 0.0) {
        goal =
            Profiler::TrapezoidProfileUnsafe(time, m_dist, m_angle, m_max_vel,
                                             m_max_acc, m_start_vel, m_end_vel);
    }
    else {
        goal = Profiler::TriProfileUnsafe(time, m_dist, m_angle, m_max_vel,
                                          m_max_acc, m_start_vel, m_end_vel);
    }
    printf(
        "ConstantArcSpline drive d %lf a %lf vel %lf acc %lf start %lf end "
        "%lf\n",
        m_dist, m_angle, m_max_vel, m_max_acc, m_start_vel, m_end_vel);
    DBStringPrintf(DB_LINE3, "lo%0.3lf ro%0.3lf", m_left_output,
                   m_right_output);

    if (goal.error) {
        printf("trap drive error\n");
        out->SetDriveOutputIPS(1.0, -1.0);
        return;
    }

    m_l_pos_pid.SetTarget(goal.linear_dist);
    m_l_vel_pid.SetTarget(goal.linear_vel);
    m_a_pos_pid.SetTarget(goal.angular_dist);
    m_a_vel_pid.SetTarget(goal.angular_vel);

    /* vel feed forward for linear term */
    double right_l_vel_ff = goal.linear_vel;
    double left_l_vel_ff = goal.linear_vel;

    /* vel feed forward for angular term */
    double right_a_vel_ff = DRIVE_WIDTH / 2.0 * RAD_PER_DEG * goal.angular_vel;
    double left_a_vel_ff = -right_a_vel_ff;

    /* correction terms for error in {linear,angular} {position,velocioty */
    double linear_dist_term = m_l_pos_pid.CalcOutput(DistFromStart());
    double linear_vel_term = m_l_vel_pid.CalcOutput(state->GetRate());
    double angular_dist_term = m_a_pos_pid.CalcOutput(AngleFromStart());
    double angular_vel_term = m_a_vel_pid.CalcOutput(state->GetAngularRate());
    // printf("angle_dist_term: %lf angle_from_start %lf angle_goal %lf\n",
    //        angular_dist_term, AngleFromStart(), goal.angular_dist);

    /* right side receives positive angle correction */
    double right_output = right_l_vel_ff + right_a_vel_ff + linear_dist_term +
                          linear_vel_term + angular_dist_term +
                          angular_vel_term;
    /* left side receives negative angle correction */
    double left_output = left_l_vel_ff + left_a_vel_ff + linear_dist_term +
                         linear_vel_term - angular_dist_term - angular_vel_term;

    out->SetDriveOutputIPS(left_output, right_output);

    m_done = goal.done;

    m_l_pos_setpt_log->LogDouble(goal.linear_dist);
    m_l_pos_real_log->LogDouble(DistFromStart());
    m_l_vel_setpt_log->LogDouble(goal.linear_vel);
    m_l_vel_real_log->LogDouble(state->GetRate());
    m_a_pos_setpt_log->LogDouble(goal.angular_dist);
    m_a_pos_real_log->LogDouble(AngleFromStart());
    m_a_vel_setpt_log->LogDouble(goal.angular_vel);
    m_max_vel_log->LogDouble(m_max_vel);
    m_max_acc_log->LogDouble(m_max_acc);
    m_dist_endgoal_log->LogDouble(m_dist);
    m_angle_endgoal_log->LogDouble(m_angle);
    m_left_output->LogDouble(left_output);
    m_right_output->LogDouble(right_output);

    // printf("ConstantArcSplineDriveController active time %lf pos %lf\n",
    // time,
    //        goal.linear_dist);
}

void ConstantArcSplineDriveController::Start(DriveControlSignalReceiver *out) {
    m_needSetControlMode = true;
}

void ConstantArcSplineDriveController::Stop(DriveControlSignalReceiver *out) {
}

double ConstantArcSplineDriveController::DistFromStart() const {
    return m_state->GetDist() - m_dist_offset;
}

double ConstantArcSplineDriveController::AngleFromStart() const {
    return m_state->GetAngle() - m_angle_offset;
}
}
