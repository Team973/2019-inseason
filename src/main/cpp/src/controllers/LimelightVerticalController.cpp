#include "src/controllers/LimelightVerticalController.h"
#include "lib/util/WrapDash.h"
#include "stdio.h"

namespace frc973 {
LimelightVerticalController::LimelightVerticalController(Limelight *limelight,
                                                         GreyTalonSRX *motor)
        : m_onTarget(false)
        , m_setpoint(0.0)
        , m_limelight(limelight)
        , m_motor(motor) {
}

LimelightVerticalController::~LimelightVerticalController() {
}

void LimelightVerticalController::Start() {
    m_limelight->SetCameraVisionCenter();
}

void LimelightVerticalController::CalcOutput() {
    double offset = m_limelight->GetYOffset();

    if (!m_limelight->isTargetValid()) {
        m_setpoint = 0.0;
    }
    else {
        m_setpoint = offset * VELOCITY_MULTIPLIER;  // degrees cancel out
    }

    m_motor->Set(ControlMode::Velocity, m_setpoint);  // in native units

    if (fabs(offset) < 5.0) {
        m_onTarget = true;
    }
    else {
        m_onTarget = false;
    }
}
}
