#include "src/controllers/LimelightDriveController.h"
#include "lib/util/WrapDash.h"
#include "stdio.h"
#include "lib/helpers/PID.h"

namespace frc973 {
LimelightDriveController::LimelightDriveController(Limelight *limelight)
        : m_onTarget(false)
        , m_leftSetpoint(0.0)
        , m_rightSetpoint(0.0)
        , m_throttle(0.0)
        , m_turn(0.0)
        , m_limelight(limelight)
        , m_turnPid(new PID(0.07, 0.0, 0.0))
        , m_throttlePid(new PID(0.1, 0.0, 0.0)) {
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

void LimelightDriveController::CalcDriveOutput(
    DriveStateProvider *state, DriveControlSignalReceiver *out) {
    double offset = m_limelight->GetXOffset();
    double distance = m_limelight->GetHorizontalDistance();  // in inches
    double distError = DISTANCE_SETPOINT - distance;
    /*if (fabs(offset) < 3.0) {
        offset = 0.0;
    }*/

    if (!m_limelight->isTargetValid() || m_onTarget) {
        m_leftSetpoint = 0.0;
        m_rightSetpoint = 0.0;
    }
    else {
        double turnPidOut = m_turnPid->CalcOutputWithError(offset);
        double throttlePidOut = m_throttlePid->CalcOutputWithError(
            distError);                    // * pow(cos(offset), 5);
        m_leftSetpoint = throttlePidOut;   // - turnPidOut;*/
        m_rightSetpoint = throttlePidOut;  //+ turnPidOut;
    }

    out->SetDriveOutputVBus(m_leftSetpoint * DRIVE_OUTPUT_MULTIPLIER,
                            -m_rightSetpoint * DRIVE_OUTPUT_MULTIPLIER);

    DBStringPrintf(DBStringPos::DB_LINE5, "LS: %3.2lf RS: %3.2lf",
                   m_leftSetpoint, m_rightSetpoint);
    DBStringPrintf(DBStringPos::DB_LINE6, "LimeError: %3.2lf",
                   m_limelight->GetXOffset());

    if ((fabs(offset) < 5.0 && fabs(state->GetAngularRate()) < 5.0) &&
        (fabs(distError) < 5.0 && fabs(state->GetRate() < 5.0))) {
        m_onTarget = true;
    }
    else {
        m_onTarget = false;
    }
}
}
