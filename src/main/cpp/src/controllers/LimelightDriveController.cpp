#include "src/controllers/LimelightDriveController.h"
#include "lib/util/WrapDash.h"
#include "stdio.h"
#include "lib/helpers/PID.h"

namespace frc973 {
LimelightDriveController::LimelightDriveController(Limelight *limelight,
                                                   VisionOffset offset,
                                                   bool isCompSkew)
        : m_onTarget(false)
        , m_leftSetpoint(0.0)
        , m_rightSetpoint(0.0)
        , m_visionOffset((offset == VisionOffset::Cargo ? CARGO_VISION_OFFSET
                                                        : HATCH_VISION_OFFSET))
        , m_isCompensatingSkew(isCompSkew)
        , m_throttle(0.0)
        , m_turn(0.0)
        , m_goalAngleComp(0.0)
        , m_limelight(limelight)
        , m_turnPid(new PID(0.02, 0.0, 0.001))
        , m_throttlePid(new PID(0.03, 0.0, 0.003)) {
}

LimelightDriveController::~LimelightDriveController() {
    delete m_turnPid, m_throttlePid;
}

void LimelightDriveController::Start(DriveControlSignalReceiver *out) {
    printf("Turning on Limelight Drive Mode\n");
    m_limelight->SetCameraVision();
    m_limelight->SetLightOn();
    m_onTarget = false;
}

double LimelightDriveController::CalcScaleGoalAngleComp() {
    return Util::bound(
        GOAL_ANGLE_COMP_KP * m_limelight->GetTargetSkew() *
            Util::bound(1 / (GOAL_ANGLE_COMP_MAX - GOAL_ANGLE_COMP_MIN) *
                                m_limelight->GetHorizontalDistance() -
                            (GOAL_ANGLE_COMP_MIN * 1 /
                             (GOAL_ANGLE_COMP_MAX - GOAL_ANGLE_COMP_MIN)),
                        0.0, 1.0),
        -0.35, 0.35);  // y = mx + b
                       // y = degree of compensation
                       // m = (1 - 0) / (max - min)
                       // x = distance to target
                       // b = y-int as plugged in to slope intercept equation
}

void LimelightDriveController::CalcDriveOutput(
    DriveStateProvider *state, DriveControlSignalReceiver *out) {
    double offset = m_limelight->GetXOffset();
    double distance = m_limelight->GetHorizontalDistance();  // in inches
    double distError = distance - DISTANCE_SETPOINT;

    if (!m_limelight->isTargetValid() || m_onTarget) {
        m_leftSetpoint = 0.0;
        m_rightSetpoint = 0.0;
    }
    else {
        double turnPidOut = Util::bound(
            -m_turnPid->CalcOutputWithError(offset - HATCH_VISION_OFFSET), -0.5,
            0.5);
        double throttlePidOut = Util::bound(
            m_throttlePid->CalcOutputWithError(
                -distError *
                (pow(cos((offset * Constants::PI / 180.0) * PERIOD), 5))),
            -0.5, 0.5);
        m_goalAngleComp = CalcScaleGoalAngleComp();
        if (m_isCompensatingSkew) {
            m_leftSetpoint = throttlePidOut + turnPidOut + m_goalAngleComp;
            m_rightSetpoint = throttlePidOut - turnPidOut - m_goalAngleComp;
        }
        else {
            m_leftSetpoint = throttlePidOut + turnPidOut;
            m_rightSetpoint = throttlePidOut - turnPidOut;
        }
    }

    out->SetDriveOutputVBus(m_leftSetpoint * DRIVE_OUTPUT_MULTIPLIER,
                            m_rightSetpoint * DRIVE_OUTPUT_MULTIPLIER);

    if ((fabs(offset) < 5.0 && fabs(state->GetAngularRate()) < 5.0) &&
        (fabs(distError) < 3.0 && fabs(state->GetRate() < 3.0))) {
        m_onTarget = true;
    }
    else {
        m_onTarget = false;
    }
}
}
