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
        , m_DBTurnPID(new PID(m_DBTurnPIDkP, m_DBTurnPIDkI, m_DBTurnPIDkD))
        , m_disableThrottlePidOut(false)
        , m_disableTurnPidOut(false)
        , m_disableSkewComp(false) {
}

LimelightDriveController::~LimelightDriveController() {
    delete m_turnPid, m_throttlePid;
}

double LimelightDriveController::GetThrottlePidOut() const {
    return m_DBThrottlePIDOut;
}

double LimelightDriveController::GetTurnPidOut() const {
    return m_DBTurnPIDOut;
}

double LimelightDriveController::GetGoalAngleComp() const {
    return m_DBGoalAngleComp;
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
                          m_distance),
        0.0, 1.0);
    double skew = m_limelight->GetTargetSkew();
    double frame_multiplier = Util::bound(
        Util::interpolate(Util::Point(SKEW_COMP_MULTIPLIER_DISTANCE_MIN, 1),
                          Util::Point(SKEW_COMP_MULTIPLIER_DISTANCE_MAX, 0),
                          fabs(m_limelight->GetXOffset())),
        0.0, 1.0);
    m_skew_comp = Util::bound(
        m_DBGoalAngleCompkP * skew * frame_multiplier * dist_multiplier,
        m_DBSkewMin, m_DBSkewMax);

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
                          horizontalDistance), 0.5, 1.0);
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
    double distance = m_limelight->GetHorizontalDistance();  // in inches
    double distError;
    if (m_hatchIntake->GetHatchPuncherState() ==
        HatchIntake::HatchSolenoidState::manualPunch) {
        distError = distance - m_DBDistanceSetpointRocket;
    }
    else {
        distError = distance - m_DBDistanceSetpointCargoBay;
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
    else {
        m_DBTurnPIDOut = Util::bound(m_DBTurnPID->CalcOutputWithError(
                                     offset - m_DBHatchVisionOffset),
                                     m_DBTurnMin, m_DBTurnMax) * CalcTurnComp(distance);

        m_DBThrottlePIDOut = Util::bound(m_DBThrottlePID->CalcOutputWithError(-distError),
                                         m_DBThrottleMin, m_DBThrottleMax);

        m_DBGoalAngleComp = CalcScaleGoalAngleComp();

        if (m_isCompensatingSkew) {
            if (m_disableThrottlePidOut == true) {
                m_DBThrottlePIDOut = 0.0;
            }

            if (m_disableTurnPidOut == true) {
                m_DBTurnPIDOut = 0.0;
            }

            if (m_disableSkewComp == true) {
                m_DBGoalAngleComp = 0.0;
            }

            m_leftSetpoint = m_DBThrottlePIDOut - m_DBTurnPIDOut - m_DBGoalAngleComp;
            m_rightSetpoint = m_DBThrottlePIDOut + m_DBTurnPIDOut + m_DBGoalAngleComp;
        }
        else {
            if (m_disableThrottlePidOut == true) {
                m_DBThrottlePIDOut = 0.0;
            }

            if (m_disableTurnPidOut == true) {
                m_DBTurnPIDOut = 0.0;
            }

            m_leftSetpoint = m_DBThrottlePIDOut + m_DBTurnPIDOut;
            m_rightSetpoint = m_DBThrottlePIDOut - m_DBTurnPIDOut;
        }
    }


    /* DBStringPrintf(DBStringPos::DB_LINE3, "th%2.2lf tu%2.2lf sk%2.2lf",
                   m_throttlePidOut, m_turnPidOut, m_goalAngleComp);
    DBStringPrintf(DBStringPos::DB_LINE4, "lim: l:%2.2lf r:%2.2lf",
                   m_leftSetpoint, m_rightSetpoint); */

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
    if (SmartDashboard::GetBoolean("DB/Button 0", false) == true) {
        m_DBThrottlePIDkP =
            stod(SmartDashboard::GetString("DB/String 0", "0.0").substr(3, 6));
        m_DBThrottlePIDkI =
            stod(SmartDashboard::GetString("DB/String 0", "0.0").substr(10, 4));
        m_DBThrottlePIDkD =
            stod(SmartDashboard::GetString("DB/String 0", "0.0").substr(15, 5));
        m_DBTurnPIDkP =
            stod(SmartDashboard::GetString("DB/String 1", "0.0").substr(3, 6));
        m_DBTurnPIDkI =
            stod(SmartDashboard::GetString("DB/String 1", "0.0").substr(10, 4));
        m_DBTurnPIDkD =
            stod(SmartDashboard::GetString("DB/String 1", "0.0").substr(15, 5));
        m_DBGoalAngleCompkP =
            stod(SmartDashboard::GetString("DB/String 2", "0.0").substr(3, 6));
        m_DBThrottleFeedForward =
            stod(SmartDashboard::GetString("DB/String 2", "0.0").substr(15, 5));
        m_DBHatchVisionOffset =
            stod(SmartDashboard::GetString("DB/String 3", "0.0").substr(3, 5));

        double offset = m_limelight->GetXOffset();

        DBStringPrintf(DB_LINE3, "HO %2.2lf XO: %2.2lf", m_DBHatchVisionOffset,
                           offset);

        m_DBDistanceSetpointRocket =
            stod(SmartDashboard::GetString("DB/String 4", "0.0").substr(7, 4));
        m_DBDistanceSetpointCargoBay =
            stod(SmartDashboard::GetString("DB/String 4", "0.0").substr(15, 5));
        m_DBThrottleMax =
            stod(SmartDashboard::GetString("DB/String 5", "0.0").substr(5, 4));
        m_DBThrottleMin = -m_DBThrottleMax;
        m_DBTurnMax =
            stod(SmartDashboard::GetString("DB/String 5", "0.0").substr(13, 4));
        m_DBTurnMin = -m_DBTurnMax;
        m_DBSkewMax =
            stod(SmartDashboard::GetString("DB/String 5", "0.0").substr(20, 4));
        m_DBSkewMin = -m_DBSkewMax;

        if (SmartDashboard::GetBoolean("DB/Button 1", false) == true) {
            m_disableThrottlePidOut = true;
        }
        else {
            m_disableThrottlePidOut = false;
        }

        if (SmartDashboard::GetBoolean("DB/Button 2", false) == true) {
            m_disableTurnPidOut = true;
        }
        else {
            m_disableTurnPidOut = false;
        }

        if (SmartDashboard::GetBoolean("DB/Button 3", false) == true) {
            m_disableSkewComp = true;
        }
        else {
            m_disableSkewComp = false;
        }

    }
}

void LimelightDriveController::CreateLimelightDriveDB() {
    // "Th 0.0180 0.00 0.002"
    DBStringPrintf(DB_LINE0, "Th %1.4lf %1.2lf %1.3lf", m_DBThrottlePIDkP, m_DBThrottlePIDkI, m_DBThrottlePIDkD);
    // "Tu 0.0120 0.00 0.003"
    DBStringPrintf(DB_LINE1, "Tu %1.4lf %1.2lf %1.3lf", m_DBTurnPIDkP, m_DBTurnPIDkI, m_DBTurnPIDkD);
    // "AC 0.0120 Feed 0.000"
    DBStringPrintf(DB_LINE2, "AC %1.4lf Feed: %2.3lf", m_DBGoalAngleCompkP, m_DBThrottleFeedForward);
    // "HO -0.70 XO: 0.00"
    DBStringPrintf(DB_LINE3, "HO %2.2lf XO: 0.00", m_DBHatchVisionOffset);
    // "D-S Ro 4.00 Ca -4.00"
    DBStringPrintf(DB_LINE4, "D-S Ro %2.2lf Ca %2.2lf", m_DBDistanceSetpointRocket, m_DBDistanceSetpointCargoBay);
    // "B th 0.60 tr 0.40 s 0.20"
    DBStringPrintf(DB_LINE5, "B th %1.2lf tr %1.2lf s %1.2lf", m_DBThrottleMax, m_DBTurnMax, m_DBSkewMax);
}
}
