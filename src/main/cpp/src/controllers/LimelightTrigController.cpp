#include "src/controllers/LimelightTrigController.h"
#include "lib/util/WrapDash.h"
#include "stdio.h"
#include "lib/helpers/PID.h"
#include <math.h>

namespace frc973 {
LimelightTrigController::LimelightTrigController(
    LogSpreadsheet *logger, Limelight *limelight,
    ObservablePoofsJoystick *driverJoystick,
    ObservableXboxJoystick *operatorJoystick, HatchIntake *hatchIntake)
        : m_onTarget(false)
        , m_leftSetpoint(0.0)
        , m_rightSetpoint(0.0)
        , m_driverJoystick(driverJoystick)
        , m_operatorJoystick(operatorJoystick)
        , m_hatchIntake(hatchIntake)
        , m_throttle(0.0)
        , m_turn(0.0)
        , m_gyroAngle(0.0)
        , m_limelight(limelight)
        , m_throttlePidOut(0.0)
        , m_turnPidOut(0.0)
        , m_skewPidOut(0.0)
        , m_turnPid(new PID(TURN_PID_KP, TURN_PID_KI, TURN_PID_KD))
        , m_throttlePid(
              new PID(THROTTLE_PID_KP, THROTTLE_PID_KI, THROTTLE_PID_KD))
        , m_skewPid(new PID(SKEW_PID_KP, SKEW_PID_KI, SKEW_PID_KD)) {
}

LimelightTrigController::~LimelightTrigController() {
    delete m_turnPid, m_throttlePid;
}

double LimelightTrigController::GetThrottlePidOut() const {
    return m_throttlePidOut;
}

double LimelightTrigController::GetTurnPidOut() const {
    return m_turnPidOut;
}

double LimelightTrigController::GetSkewPidOut() const {
    return m_skewPidOut;
}

void LimelightTrigController::Start(DriveControlSignalReceiver *out) {
    printf("Turning on Limelight Drive Mode\n");
    m_limelight->SetCameraVisionCenter();
    m_limelight->SetLightOn();
    m_onTarget = false;
}

double LimelightTrigController::CalcTurnComp() {
    return Util::bound(
        Util::interpolate(Util::Point(TURN_COMP_DISTANCE_MIN, 0.5),
                          Util::Point(TURN_COMP_DISTANCE_MAX, 1.0),
                          m_limelight->GetHorizontalDistance()),
        0.5, 1.0);
}

double LimelightTrigController::GetTargetDirectionConstant() {
    if (m_hatchIntake->GetHatchPuncherState() ==
            HatchIntake::HatchSolenoidState::manualPunch ||
        m_operatorJoystick->GetRawButton(Xbox::BtnY)) {
        if (m_driverJoystick->GetRawAxisWithDeadband(
                PoofsJoysticks::RightXAxis) > 0.5) {
            if (fabs(m_gyroAngle) > 90.0) {
                return LEFT_BACK_ROCKET;
            }
            else {
                return LEFT_FRONT_ROCKET;
            }
        }
        else {
            if (fabs(m_gyroAngle) > 90) {
                return RIGHT_BACK_ROCKET;
            }
            else {
                return RIGHT_FRONT_ROCKET;
            }
        }
    }
    else {
        int gyroRounded = round(m_gyroAngle / 90) * 90;
        switch (gyroRounded) {
            case 0:
                return FRONT_CARGO;
                break;
            case 90:
                return RIGHT_CARGO_BAY;
                break;
            case -90:
                return LEFT_CARGO_BAY;
                break;
            case 180:
            case -180:
            default:
                return HUMAN_LOADING_STATION;
                break;
        }
    }
}

void LimelightTrigController::CalcDriveOutput(DriveStateProvider *state,
                                              DriveControlSignalReceiver *out) {
    if (m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::RightXAxis) >
        0.5) {
        m_limelight->SetCameraVisionLeft();
    }
    else if (m_driverJoystick->GetRawAxisWithDeadband(
                 PoofsJoysticks::RightXAxis) < -0.5) {
        m_limelight->SetCameraVisionRight();
    }
    m_limelight->SetLightOn();
    double limelight_offset = m_limelight->GetXOffset();
    double distance = m_limelight->GetHorizontalDistance();  // in inches
    double distError;
    m_gyroAngle = state->GetAngle();
    double skewAngle =
        180.0 - GetTargetDirectionConstant() - limelight_offset + m_gyroAngle;
    if (m_hatchIntake->GetHatchPuncherState() ==
        HatchIntake::HatchSolenoidState::manualPunch) {
        distError = distance - DISTANCE_SETPOINT_ROCKET;
    }
    else {
        distError = distance - DISTANCE_SETPOINT_CARGO_BAY;
    }

    if (!m_limelight->isTargetValid() || m_onTarget) {
        // Proof on concept: Allow driver to turn to get a target, should only
        // be when !isTargetValid(), so break away from the || above
        m_leftSetpoint = 0.0;   //- driverComp;
        m_rightSetpoint = 0.0;  //+ driverComp;
    }
    else {
        m_turnPidOut = Util::bound(m_turnPid->CalcOutputWithError(
                                       limelight_offset - HATCH_VISION_OFFSET),
                                   TURN_MIN, TURN_MAX) *
                       CalcTurnComp();
        m_throttlePidOut =
            Util::bound(m_throttlePid->CalcOutputWithError(-distError),
                        THROTTLE_MIN, THROTTLE_MAX);
        m_skewPidOut = Util::bound(m_skewPid->CalcOutputWithError(skewAngle),
                                   SKEW_MIN, SKEW_MAX);

        m_leftSetpoint = m_throttlePidOut - m_turnPidOut - m_skewPidOut;
        m_rightSetpoint = m_throttlePidOut + m_turnPidOut + m_skewPidOut;
    }
    DBStringPrintf(DBStringPos::DB_LINE3, "th%2.2lf tu%2.2lf sk%2.2lf",
                   m_throttlePidOut, m_turnPidOut, m_skewPidOut);
    DBStringPrintf(DBStringPos::DB_LINE4, "lim: l:%2.2lf r:%2.2lf",
                   m_leftSetpoint, m_rightSetpoint);

    out->SetDriveOutputVBus(m_leftSetpoint, m_rightSetpoint);

    if ((fabs(limelight_offset) < 5.0 && fabs(state->GetAngularRate()) < 5.0) &&
        (fabs(distError) < 3.0 && fabs(state->GetRate() < 3.0))) {
        m_onTarget = true;
    }
    else {
        m_onTarget = false;
    }
}
}
