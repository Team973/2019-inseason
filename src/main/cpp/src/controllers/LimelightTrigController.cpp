#include "src/controllers/LimelightTrigController.h"
#include "lib/util/WrapDash.h"
#include "stdio.h"
#include "lib/helpers/PID.h"

namespace frc973 {
LimelightTrigController::LimelightTrigController(
    LogSpreadsheet *logger, Limelight *limelight, bool isCompSkew,
    ObservablePoofsJoystick *driverJoystick, HatchIntake *hatchIntake)
        : m_onTarget(false)
        , m_leftSetpoint(0.0)
        , m_rightSetpoint(0.0)
        , m_isCompensatingSkew(isCompSkew)
        , m_driverJoystick(driverJoystick)
        , m_hatchIntake(hatchIntake)
        , m_throttle(0.0)
        , m_turn(0.0)
        , m_limelight(limelight)
        , m_throttlePidOut(0.0)
        , m_turnPidOut(0.0)
        , m_goalAngleComp(0.0)
        , m_turnPid(new PID(TURN_PID_KP, TURN_PID_KI, TURN_PID_KD))
        , m_throttlePid(
              new PID(THROTTLE_PID_KP, THROTTLE_PID_KI, THROTTLE_PID_KD)) {
}

LimelightTrigController::~LimelightTrigController() {
    delete m_turnPid, m_throttlePid;
}

double LimelightTrigController::CalcScaleGoalAngleComp() {
    double dist_multiplier = Util::bound(
        Util::interpolate(Util::Point(GOAL_ANGLE_COMP_DISTANCE_MIN, 0),
                          Util::Point(GOAL_ANGLE_COMP_DISTANCE_MAX, 1),
                          m_limelight->GetHorizontalDistance()),
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

double LimelightTrigController::GetThrottlePidOut() const {
    return m_throttlePidOut;
}

double LimelightTrigController::GetTurnPidOut() const {
    return m_turnPidOut;
}

double LimelightTrigController::GetGoalAngleComp() const {
    return m_goalAngleComp;
}

void LimelightTrigController::Start(DriveControlSignalReceiver *out) {
    printf("Turning on Limelight Drive Mode\n");
    m_limelight->SetCameraVisionCenter();
    m_limelight->SetLightOn();
    m_onTarget = false;
}

void LimelightTrigController::CalcDriveOutput(DriveStateProvider *state,
                                              DriveControlSignalReceiver *out) {
    double offset = m_limelight->GetXOffset() * Constants::RAD_PER_DEG;
    double gyro_angle = state->GetAngle() * Constants::RAD_PER_DEG;
    double distance = m_limelight->GetHorizontalDistance();  // in inches
    double distError;
    double skewAngle = gyro_angle - offset;
    double theta = skewAngle - offset;

    m_throttle = cos(theta);
    m_turn = -sin(theta);

    m_leftSetpoint = m_throttle - m_turn;
    m_rightSetpoint = m_throttle + m_turn;

    out->SetDriveOutputVBus(m_leftSetpoint, m_rightSetpoint);
}
}
