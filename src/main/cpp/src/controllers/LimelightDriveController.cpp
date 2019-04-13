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
        , m_limelight(limelight)
        , m_throttlePidOut(0.0)
        , m_turnPidOut(0.0)
        , m_goalAngleComp(0.0)
        , m_turnPid(new PID(TURN_PID_KP, TURN_PID_KI, TURN_PID_KD))
        , m_throttlePid(
              new PID(THROTTLE_PID_KP, THROTTLE_PID_KI, THROTTLE_PID_KD))
        , m_DBThrottlePIDkP(THROTTLE_PID_KP)
        , m_DBThrottlePIDkI(THROTTLE_PID_KI)
        , m_DBThrottlePIDkD(THROTTLE_PID_KD)
        , m_DBThrottlePIDOut(0.0)
        , m_DBTurnPIDkP(TURN_PID_KP)
        , m_DBTurnPIDkI(TURN_PID_KI)
        , m_DBTurnPIDkD(TURN_PID_KD)
        , m_DBTurnPIDOut(0.0)
        , m_DBGoalAngleCompkP(GOAL_ANGLE_COMP_KP)
        , m_DBGoalAngleComp(0.0)
        , m_DBThrottleFeedForward(THROTTLE_FEED_FORWARD)
        , m_DBHatchVisionOffset(HATCH_VISION_OFFSET)
        , m_DBHatchVisionXOffset(0.0)
        , m_DBDistanceSetpointRocket(DISTANCE_SETPOINT_ROCKET)
        , m_DBDistanceSetpointCargoBay(DISTANCE_SETPOINT_CARGO_BAY)
        , m_DBThrottleMin(THROTTLE_MIN)
        , m_DBThrottleMax(THROTTLE_MAX)
        , m_DBTurnMin(TURN_MIN)
        , m_DBTurnMax(TURN_MAX)
        , m_DBSkewMin(SKEW_MIN)
        , m_DBSkewMax(SKEW_MAX)
        , m_DBThrottlePID(
              new PID(m_DBThrottlePIDkP, m_DBThrottlePIDkI, m_DBThrottlePIDkD))
        , m_DBTurnPID(new PID(m_DBTurnPIDkP, m_DBTurnPIDkI, m_DBTurnPIDkD)) {
}

LimelightDriveController::~LimelightDriveController() {
    delete m_turnPid, m_throttlePid;
}

double LimelightDriveController::GetThrottlePidOut() const {
    if (SmartDashboard::GetBoolean("DB/Button 0", false) == true) {
        return m_DBThrottlePIDOut;
    }
    else {
        return m_throttlePidOut;
    }
}

double LimelightDriveController::GetTurnPidOut() const {
    if (SmartDashboard::GetBoolean("DB/Button 0", false) == true) {
        return m_DBTurnPIDOut;
    }
    else {
        return m_turnPidOut;
    }
}

double LimelightDriveController::GetGoalAngleComp() const {
    if (SmartDashboard::GetBoolean("DB/Button 0", false) == true) {
        return m_DBGoalAngleComp;
    }
    else {
        return m_goalAngleComp;
    }
}

void LimelightDriveController::Start(DriveControlSignalReceiver *out) {
    printf("Turning on Limelight Drive Mode\n");
    m_limelight->SetCameraVisionCenter();
    m_limelight->SetLightOn();
    m_onTarget = false;

    // Reset the PID controller(forget any stateful information), helpful when setting a new unrelated setpoint
    m_throttlePid->Reset(NAN);
    m_turnPid->Reset(NAN);

    m_DBThrottlePID->SetGains(m_DBThrottlePIDkP, m_DBThrottlePIDkI,
                              m_DBThrottlePIDkD);
    m_DBTurnPID->SetGains(m_DBTurnPIDkP, m_DBTurnPIDkI, m_DBTurnPIDkD);
    m_DBThrottlePID->Reset(NAN);
    m_DBTurnPID->Reset(NAN);
}

double LimelightDriveController::CalcScaleGoalAngleComp() {
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
    if (SmartDashboard::GetBoolean("DB/Button 0", false) == true) {
        m_skew_comp = Util::bound(
            m_DBGoalAngleCompkP * skew * frame_multiplier * dist_multiplier,
            m_DBSkewMin, m_DBSkewMax);
    }
    else {
        m_skew_comp = Util::bound(
            GOAL_ANGLE_COMP_KP * skew * frame_multiplier * dist_multiplier,
            SKEW_MIN, SKEW_MAX);
    }
    return -m_skew_comp;  // y = mx + b
                          // y = degree of compensation
                          // m = (1 - 0) / (max - min)
                          // x = distance to target
                          // b = y-int as plugged in to slope intercept equation
}

double LimelightDriveController::CalcTurnComp(double horizontalDistance) {
    return Util::bound(
        Util::interpolate(Util::Point(TURN_COMP_DISTANCE_MIN, 0.5),
                          Util::Point(TURN_COMP_DISTANCE_MAX, 1.0),
                          horizontalDistance),
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

    if (SmartDashboard::GetBoolean("DB/Button 0", false) == true) {
        DBStringPrintf(DB_LINE3, "HO: %2.2lf XO: %2.2lf", m_DBHatchVisionOffset, offset);
    }

    double distance = m_limelight->GetHorizontalDistance();  // in inches
    double distError;
    if (m_hatchIntake->GetHatchPuncherState() ==
        HatchIntake::HatchSolenoidState::manualPunch) {
        if (SmartDashboard::GetBoolean("DB/Button 0", false) == true) {
            distError = distance - m_DBDistanceSetpointRocket;
        }
        else {
            distError = distance - DISTANCE_SETPOINT_ROCKET;
        }
    }
    else {
        if (SmartDashboard::GetBoolean("DB/Button 0", false) == true) {
            distError = distance - m_DBDistanceSetpointCargoBay;
        }
        else {
            distError = distance - DISTANCE_SETPOINT_CARGO_BAY;
        }
    }

    if (!m_limelight->isTargetValid() || m_onTarget) {
        // Proof of concept: Allow driver to turn to get a target, should only
        // be when !isTargetValid(), so break away from the || above
        // double driverComp =
        //    0.1 *
        //    -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::RightXAxis);

        m_leftSetpoint = 0.0;   //- driverComp;
        m_rightSetpoint = 0.0;  //+ driverComp;
    }
    else if (SmartDashboard::GetBoolean("DB/Button 0", false) == true) {
        m_DBTurnPIDOut = Util::bound(m_DBTurnPID->CalcOutputWithError(
                                            offset - m_DBHatchVisionOffset),
                                        m_DBTurnMin, m_DBTurnMax) *
                            CalcTurnComp(distance);

        m_DBThrottlePIDOut =
            Util::bound(m_DBThrottlePID->CalcOutputWithError(-distError),
                        m_DBThrottleMin, m_DBThrottleMax);

        m_DBGoalAngleComp = CalcScaleGoalAngleComp();

        if (m_isCompensatingSkew) {
            if (SmartDashboard::GetBoolean("DB/Button 1", false) == true) {
                m_DBThrottlePIDOut = 0.0;
            }
            if (SmartDashboard::GetBoolean("DB/Button 2", false) == true) {
                m_DBTurnPIDOut = 0.0;
            }
            if (SmartDashboard::GetBoolean("DB/Button 3", false) == true) {
                m_DBGoalAngleComp = 0.0;
            }
            m_leftSetpoint = 
                m_DBThrottlePIDOut - m_DBTurnPIDOut -
                m_DBGoalAngleComp;
            m_rightSetpoint =
                m_DBThrottlePIDOut + m_DBTurnPIDOut +
                m_DBGoalAngleComp;
        }
        else {
            if (SmartDashboard::GetBoolean("DB/Button 1", false) == true) {
                m_DBThrottlePIDOut = 0.0;
            }
            if (SmartDashboard::GetBoolean("DB/Button 2", false) == true) {
                m_DBTurnPIDOut = 0.0;
            }
            m_leftSetpoint = m_DBThrottlePIDOut + m_DBTurnPIDOut;
            m_rightSetpoint = m_DBThrottlePIDOut - m_DBTurnPIDOut;
        }
    }
    else {
        m_turnPidOut = Util::bound(m_turnPid->CalcOutputWithError(
                                        offset - HATCH_VISION_OFFSET),
                                    TURN_MIN, TURN_MAX) *
                        CalcTurnComp(distance);
        m_throttlePidOut =
            Util::bound(m_throttlePid->CalcOutputWithError(-distError),
                        THROTTLE_MIN, THROTTLE_MAX);
        m_goalAngleComp = CalcScaleGoalAngleComp();
        if (m_isCompensatingSkew) {
            m_leftSetpoint =  // turnPidOut + m_goalAngleComp;
                m_throttlePidOut - m_turnPidOut -
                m_goalAngleComp;  // - driverComp;
            m_rightSetpoint =     //-turnPidOut - m_goalAngleComp;
                m_throttlePidOut + m_turnPidOut +
                m_goalAngleComp;  // - driverComp;
        }
        else {
            m_leftSetpoint = m_throttlePidOut + m_turnPidOut;
            m_rightSetpoint = m_throttlePidOut - m_turnPidOut;
        }
    }

    /*DBStringPrintf(DBStringPos::DB_LINE3, "th%2.2lf tu%2.2lf sk%2.2lf",
                   m_throttlePidOut, m_turnPidOut, m_goalAngleComp);
    DBStringPrintf(DBStringPos::DB_LINE4, "lim: l:%2.2lf r:%2.2lf",
                   m_leftSetpoint, m_rightSetpoint);*/

    out->SetDriveOutputVBus(m_leftSetpoint, m_rightSetpoint);

    if ((fabs(offset) < 5.0 && fabs(state->GetAngularRate()) < 5.0) &&
        (fabs(distError) < 3.0 && fabs(state->GetRate() < 3.0))) {
        m_onTarget = true;
    }
    else {
        m_onTarget = false;
    }
}

void LimelightDriveController::UpdateLimelightDriveDB() {
    m_DBThrottlePIDkP =
        stod(SmartDashboard::GetString("DB/String 0", "0.0").substr(3, 8));
    m_DBThrottlePIDkI =
        stod(SmartDashboard::GetString("DB/String 0", "0.0").substr(10, 13));
    m_DBThrottlePIDkD =
        stod(SmartDashboard::GetString("DB/String 0", "0.0").substr(15, 20));
    m_DBTurnPIDkP =
        stod(SmartDashboard::GetString("DB/String 1", "0.0").substr(3, 8));
    m_DBTurnPIDkI =
        stod(SmartDashboard::GetString("DB/String 1", "0.0").substr(10, 13));
    m_DBTurnPIDkD =
        stod(SmartDashboard::GetString("DB/String 1", "0.0").substr(15, 20));
    m_DBGoalAngleCompkP =
        stod(SmartDashboard::GetString("DB/String 2", "0.0").substr(3, 8));
    m_DBThrottleFeedForward =
        stod(SmartDashboard::GetString("DB/String 2", "0.0").substr(15, 20));
    m_DBHatchVisionOffset =
        stod(SmartDashboard::GetString("DB/String 3", "0.0").substr(3, 8));
    //m_DBHatchVisionXOffset =
    //    stod(SmartDashboard::GetString("DB/String 3", "0.0").substr(13, 17));
    m_DBDistanceSetpointRocket =
        stod(SmartDashboard::GetString("DB/String 4", "0.0").substr(6, 11));
    m_DBDistanceSetpointCargoBay =
        stod(SmartDashboard::GetString("DB/String 4", "0.0").substr(15, 20));
    m_DBThrottleMin =
        -stod(SmartDashboard::GetString("DB/String 5", "0.0").substr(4, 8));
    m_DBThrottleMax = -m_DBThrottleMin;

    m_DBTurnMin =
        -stod(SmartDashboard::GetString("DB/String 5", "0.0").substr(12, 16));
    m_DBTurnMax = -m_DBTurnMin;
    m_DBSkewMin =
        -stod(SmartDashboard::GetString("DB/String 5", "0.0").substr(19, 23));
    m_DBSkewMax = -m_DBSkewMin;
}

void LimelightDriveController::CreateLimelightDriveDB() {
    DBStringPrintf(DB_LINE0, "Th %1.4lf %1.2lf %1.3lf", m_DBThrottlePIDkP, m_DBThrottlePIDkI, m_DBThrottlePIDkD);
    //DBStringPrintf(DB_LINE0, "Th 0.0180 0.00 0.002");
    DBStringPrintf(DB_LINE1, "Tu %1.4lf %1.2lf %1.3lf", m_DBTurnPIDkP, m_DBTurnPIDkI, m_DBTurnPIDkD);
    //DBStringPrintf(DB_LINE1, "Tu 0.0120 0.00 0.003");
    DBStringPrintf(DB_LINE2, "AC:%1.4lf Feed: %2.3lf", m_DBGoalAngleCompkP, m_DBThrottleFeedForward);
    //DBStringPrintf(DB_LINE2, "AC:0.0120 Feed:+0.000");
    DBStringPrintf(DB_LINE3, "HO: %2.2lf XO: 0.00", m_DBHatchVisionOffset);
    //DBStringPrintf(DB_LINE3, "HO: -0.70 XO: 0.00");
    DBStringPrintf(DB_LINE4, "D-S Ro: %2.2lf Ca:%2.2lf", m_DBDistanceSetpointRocket, m_DBDistanceSetpointCargoBay);
    //DBStringPrintf(DB_LINE4, "D-S Ro:+4.00 Ca:-4.00");
    DBStringPrintf(DB_LINE5, "B th %1.2lf tr %1.2lf s %1.2lf", m_DBThrottleMax, m_DBTurnMax, m_DBSkewMax);
    //DBStringPrintf(DB_LINE5, "B th 0.60 tr 0.40 s 0.20");
}
}
