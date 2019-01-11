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
        , m_pid(new PID(0.5, 0.0, 0.0)) {
}

LimelightDriveController::~LimelightDriveController() {
    delete m_pid;
}

void LimelightDriveController::Start(DriveControlSignalReceiver *out) {
    m_limelight->SetCameraVision();
    m_limelight->SetLightOn();
    m_onTarget = false;
}

void LimelightDriveController::CalcDriveOutput(
    DriveStateProvider *state, DriveControlSignalReceiver *out) {
    double offset = m_limelight->GetXOffset();

    if (!m_limelight->isTargetValid()) {
        m_leftSetpoint = 0.0;
        m_rightSetpoint = 0.0;
    }
    else {
        double pidOut = m_pid->CalcOutputWithError(offset);
        m_leftSetpoint = (-pidOut + m_throttle - m_turn);
        m_rightSetpoint = (pidOut + m_throttle + m_turn);
    }

    out->SetDriveOutputIPS(-m_leftSetpoint * DRIVE_OUTPUT_MULTIPLIER,
                           -m_rightSetpoint * DRIVE_OUTPUT_MULTIPLIER);

    if (fabs(offset) < 5.0 && fabs(state->GetAngularRate()) < 1.0) {
        m_onTarget = true;
    }
    else {
        m_onTarget = false;
    }
}

void LimelightDriveController::SetJoysticks(double throttle, double turn) {
    m_throttle = throttle;
    m_turn = turn;
}
}
