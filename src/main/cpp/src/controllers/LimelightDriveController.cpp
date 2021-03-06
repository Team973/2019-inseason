#include "src/controllers/LimelightDriveController.h"

namespace frc973 {
LimelightDriveController::LimelightDriveController(
    LogSpreadsheet *logger, Limelight *limelight, bool isCompSkew,
    ObservablePoofsJoystick *driverJoystick, HatchIntake *hatchIntake,
    Elevator *elevator)
        : m_onTarget(false)
        , m_leftSetpoint(0.0)
        , m_rightSetpoint(0.0)
        , m_isCompensatingSkew(isCompSkew)
        , m_distance(0.0)
        , m_driverJoystick(driverJoystick)
        , m_hatchIntake(hatchIntake)
        , m_elevator(elevator)
        , m_throttle(0.0)
        , m_turn(0.0)
        , m_limelight(limelight)
        , m_scoreMode(Elevator::RocketScoreMode::low)
        , m_throttlePidOut(0.0)
        , m_turnPidOut(0.0)
        , m_goalAngleComp(0.0)
        , m_turnPid(new PID(TURN_PID_KP, TURN_PID_KI, TURN_PID_KD))
        , m_throttlePid(
              new PID(THROTTLE_PID_KP, THROTTLE_PID_KI, THROTTLE_PID_KD)) {
}

LimelightDriveController::~LimelightDriveController() {
    delete m_turnPid, m_throttlePid;
}

double LimelightDriveController::GetThrottlePidOut() const {
    return m_throttlePidOut;
}

double LimelightDriveController::GetTurnPidOut() const {
    return m_turnPidOut;
}

double LimelightDriveController::GetGoalAngleComp() const {
    return m_goalAngleComp;
}

void LimelightDriveController::Start(DriveControlSignalReceiver *out) {
    printf("Turning on Limelight Drive Mode\n");
    m_limelight->SetCameraVisionCenter();
    m_limelight->SetLightOn();
    m_onTarget = false;
}

double LimelightDriveController::CalcScaleGoalAngleComp() {
    double dist_multiplier = Util::bound(
        Util::interpolate(Util::Point(GOAL_ANGLE_COMP_DISTANCE_MIN, 0),
                          Util::Point(GOAL_ANGLE_COMP_DISTANCE_MAX, 1),
                          m_distance),
        0.0, 1.0);
    double skew = m_limelight->GetTargetSkew();
    double frame_multiplier = Util::bound(
        Util::interpolate(Util::Point(SKEW_COMP_MULTIPLIER_DISTANCE_MIN, 1),
                          Util::Point(SKEW_COMP_MULTIPLIER_DISTANCE_MAX, 0),
                          fabs(m_limelight->GetXOffset())),
        0.0, 1.0);
    double skew_comp = Util::bound(
        GOAL_ANGLE_COMP_KP * skew * frame_multiplier * dist_multiplier,
        SKEW_MIN, SKEW_MAX);
    return -skew_comp;  // y = mx + b
                        // y = degree of compensation
                        // m = (1 - 0) / (max - min)
                        // x = distance to target
                        // b = y-int as plugged in to slope intercept equation
}

double LimelightDriveController::CalcTurnComp() {
    return Util::bound(
        Util::interpolate(Util::Point(TURN_COMP_DISTANCE_MIN, 0.5),
                          Util::Point(TURN_COMP_DISTANCE_MAX, 1.0), m_distance),
        0.5, 1.0);
}

void LimelightDriveController::CalcDriveOutput(
    DriveStateProvider *state, DriveControlSignalReceiver *out) {
    if (m_driverJoystick->GetRawAxisWithDeadband(PoofsJoystick::RightXAxis) >
        0.5) {
        m_limelight->SetCameraVisionLeft();
    }
    else if (m_driverJoystick->GetRawAxisWithDeadband(
                 PoofsJoystick::RightXAxis) < -0.5) {
        m_limelight->SetCameraVisionRight();
    }
    m_limelight->SetLightOn();
    double offset = m_limelight->GetXOffset();
    m_distance = m_limelight->GetHorizontalDistance();  // in inches
    double distError;
    if (m_hatchIntake->GetHatchPuncherState() ==
            HatchIntake::HatchSolenoidState::manualPunch ||
        m_elevator->GetRocketScoreMode() == Elevator::RocketScoreMode::middle) {
        distError = m_distance - DISTANCE_SETPOINT_ROCKET;
    }
    else {
        distError = m_distance - DISTANCE_SETPOINT_CARGO_BAY;
    }

    if (!m_limelight->isTargetValid() || m_onTarget) {
        // Proof of concept: Allow driver to turn to get a target, should only
        // be when !isTargetValid(), so break away from the || above
        m_leftSetpoint = 0.0;   //- driverComp;
        m_rightSetpoint = 0.0;  //+ driverComp;
    }
    else {
        m_turnPidOut =
            Util::bound(
                m_turnPid->CalcOutputWithError(offset - HATCH_VISION_OFFSET),
                TURN_MIN, TURN_MAX) *
            CalcTurnComp();
        m_throttlePidOut =
            Util::bound(m_throttlePid->CalcOutputWithError(-distError),
                        THROTTLE_MIN, THROTTLE_MAX);
        m_goalAngleComp = CalcScaleGoalAngleComp();
        if (m_isCompensatingSkew) {
            m_leftSetpoint = m_throttlePidOut - m_turnPidOut - m_goalAngleComp;
            m_rightSetpoint = m_throttlePidOut + m_turnPidOut + m_goalAngleComp;
        }
        else {
            m_leftSetpoint = m_throttlePidOut + m_turnPidOut;
            m_rightSetpoint = m_throttlePidOut - m_turnPidOut;
        }
    }
    DBStringPrintf(DBStringPos::DB_LINE3, "th%2.2lf tu%2.2lf sk%2.2lf",
                   m_throttlePidOut, m_turnPidOut, m_goalAngleComp);
    DBStringPrintf(DBStringPos::DB_LINE4, "lim: l:%2.2lf r:%2.2lf",
                   m_leftSetpoint, m_rightSetpoint);

    out->SetDriveOutputVBus(m_leftSetpoint, m_rightSetpoint);

    if ((fabs(offset) < 5.0 && fabs(state->GetAngularRate()) < 5.0) &&
        (fabs(distError) < 3.0 && fabs(state->GetRate() < 3.0))) {
        m_onTarget = true;
    }
    else {
        m_onTarget = false;
    }
}
}
