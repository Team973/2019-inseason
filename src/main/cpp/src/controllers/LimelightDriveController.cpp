#include "src/controllers/LimelightDriveController.h"
#include "lib/util/WrapDash.h"
#include "stdio.h"
#include "lib/helpers/PID.h"

namespace frc973 {
LimelightDriveController::LimelightDriveController(
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
        , m_goalAngleComp(0.0)
        , m_limelight(limelight)
        , m_targetLog(new LogCell("LL Target Valid?"))
        , m_xOffsetLog(new LogCell("LL X Offset"))
        , m_yOffsetLog(new LogCell("LL Y Offset"))
        , m_targetAreaLog(new LogCell("LL Target Area"))
        , m_targetSkewLog(new LogCell("LL Target Skew"))
        , m_latencyLog(new LogCell("LL Latency"))
        , m_pipelineLog(new LogCell("LL Pipeline"))
        , m_horizontalLengthLog(new LogCell("LL Horizontal Length"))
        , m_verticalLengthLog(new LogCell("LL Vertical Length"))
        , m_horizontalDistanceLog(new LogCell("LL Horizontal Distance"))
        , m_turnPidErrorLog(new LogCell("LL Turn Pid Error"))
        , m_throttlePidErrorLog(new LogCell("LL Throttle Pid Error"))
        , m_leftPidSetpointLog(new LogCell("LL Left Pid Setpoint"))
        , m_rightPidSetpointLog(new LogCell("LL Right Turn Pid Setpoint"))
        , m_turnPid(new PID(0.015, 0.0, 0.002))
        , m_throttlePid(new PID(0.023, 0.0, 0.003)) {
    logger->RegisterCell(m_targetLog);
    logger->RegisterCell(m_xOffsetLog);
    logger->RegisterCell(m_yOffsetLog);
    logger->RegisterCell(m_targetAreaLog);
    logger->RegisterCell(m_targetSkewLog);
    logger->RegisterCell(m_latencyLog);
    logger->RegisterCell(m_pipelineLog);
    logger->RegisterCell(m_horizontalLengthLog);
    logger->RegisterCell(m_verticalLengthLog);
    logger->RegisterCell(m_horizontalDistanceLog);
    logger->RegisterCell(m_turnPidErrorLog);
    logger->RegisterCell(m_throttlePidErrorLog);
    logger->RegisterCell(m_leftPidSetpointLog);
    logger->RegisterCell(m_rightPidSetpointLog);
}

LimelightDriveController::~LimelightDriveController() {
    delete m_turnPid, m_throttlePid;
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
                          m_limelight->GetHorizontalDistance()),
        0.0, 1.0);
    double skew = m_limelight->GetTargetSkew();
    double skew_multiplier =
        Util::bound(Util::interpolate(Util::Point(17, 1), Util::Point(24, 0),
                                      fabs(m_limelight->GetXOffset())),
                    0.0, 1.0);
    double angle_comp = Util::bound(
        GOAL_ANGLE_COMP_KP * skew * dist_multiplier * skew_multiplier, -0.2,
        0.2);
    return angle_comp;  // y = mx + b
                        // y = degree of compensation
                        // m = (1 - 0) / (max - min)
                        // x = distance to target
                        // b = y-int as plugged in to slope intercept equation
}

double LimelightDriveController::CalcTurnComp() {
    return Util::bound(
        Util::interpolate(Util::Point(TURN_COMP_DISTANCE_MIN, 0.5),
                          Util::Point(TURN_COMP_DISTANCE_MAX, 1.0),
                          m_limelight->GetHorizontalDistance()),
        0.5, 1.0);
}

double LimelightDriveController::CalcThrottleCap() {
    /*
    return THROTTLE_MIN +
           (1.0 / (THROTTLE_CAP_DISTANCE_MAX - THROTTLE_CAP_DISTANCE_MIN) *
                m_limelight->GetHorizontalDistance() -
            (THROTTLE_CAP_DISTANCE_MIN * 1 /
             (THROTTLE_CAP_DISTANCE_MAX - THROTTLE_CAP_DISTANCE_MIN))) *
               (THROTTLE_MAX - THROTTLE_MIN);
               */
}

void LimelightDriveController::CalcDriveOutput(
    DriveStateProvider *state, DriveControlSignalReceiver *out) {
    if (m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::RightXAxis) >
        0.5) {
        m_limelight->SetCameraVisionLeft();
    }
    else if (m_driverJoystick->GetRawAxisWithDeadband(
                 PoofsJoysticks::RightXAxis) < -0.5) {
        m_limelight->SetCameraVisionRight();
    }
    m_limelight->SetLightOn();
    double offset = m_limelight->GetXOffset();
    double distance = m_limelight->GetHorizontalDistance();  // in inches
    double distError;
    if (m_hatchIntake->GetHatchPuncherState() ==
        HatchIntake::HatchSolenoidState::manualPunch) {
        distError = distance - DISTANCE_SETPOINT_ROCKET;
    }
    else {
        distError = distance - DISTANCE_SETPOINT_CARGO_BAY;
    }

    if (!m_limelight->isTargetValid() || m_onTarget) {
        m_leftSetpoint = 0.0;
        m_rightSetpoint = 0.0;
    }
    else {
        double turnPidOut =
            Util::bound(
                -m_turnPid->CalcOutputWithError(offset - HATCH_VISION_OFFSET),
                -0.4, 0.4) *
            CalcTurnComp();
        double throttlePidOut =
            Util::bound(m_throttlePid->CalcOutputWithError(-distError), -0.72,
                        0.72);  //(pow(cos((offset * Constants::PI / 180.0) *
                                // PERIOD), 5))),
        m_goalAngleComp = CalcScaleGoalAngleComp();
        double driverComp = 0.1 * m_driverJoystick->GetRawAxisWithDeadband(
                                      PoofsJoysticks::LeftYAxis);
        if (m_isCompensatingSkew) {
            m_leftSetpoint =
                throttlePidOut + turnPidOut + m_goalAngleComp;  // - driverComp;
            m_rightSetpoint =
                throttlePidOut - turnPidOut - m_goalAngleComp;  // - driverComp;
        }
        else {
            m_leftSetpoint = throttlePidOut + turnPidOut;
            m_rightSetpoint = throttlePidOut - turnPidOut;
        }
    }
    DBStringPrintf(DBStringPos::DB_LINE4, "lim: l:%2.2lf r:%2.2lf",
                   m_leftSetpoint, m_rightSetpoint);

    m_targetLog->LogDouble(m_limelight->isTargetValid());
    m_xOffsetLog->LogDouble(m_limelight->GetXOffset());
    m_yOffsetLog->LogDouble(m_limelight->GetYOffset());
    m_targetAreaLog->LogDouble(m_limelight->GetTargetArea());
    m_targetSkewLog->LogDouble(m_limelight->GetTargetSkew());
    m_latencyLog->LogDouble(m_limelight->GetLatency());
    m_pipelineLog->LogDouble(m_limelight->GetPipeline());
    m_horizontalLengthLog->LogDouble(m_limelight->GetHorizontalLength());
    m_verticalLengthLog->LogDouble(m_limelight->GetVerticalLength());
    m_horizontalDistanceLog->LogDouble(m_limelight->GetHorizontalDistance());
    m_turnPidErrorLog->LogDouble(offset - HATCH_VISION_OFFSET);
    m_throttlePidErrorLog->LogDouble(-distError);
    m_leftPidSetpointLog->LogDouble(m_leftSetpoint);
    m_rightPidSetpointLog->LogDouble(m_rightSetpoint);

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
