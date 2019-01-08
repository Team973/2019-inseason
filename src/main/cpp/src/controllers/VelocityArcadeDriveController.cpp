/*
 * VelocityArcadeDriveController.cpp
 *
 *  Created on: Feb 4, 2018
 *      Author: Kyle
 */

#include "src/controllers/VelocityArcadeDriveController.h"
#include "lib/util/Util.h"
#include <stdio.h>
#include "lib/util/WrapDash.h"
#include "src/info/RobotInfo.h"

using namespace frc;
using namespace ctre;

namespace frc973 {

VelocityArcadeDriveController::VelocityArcadeDriveController()
        : m_leftOutput(0.0), m_rightOutput(0.0) {
}

VelocityArcadeDriveController::~VelocityArcadeDriveController() {
}

void VelocityArcadeDriveController::CalcDriveOutput(
    DriveStateProvider *state, DriveControlSignalReceiver *out) {
    out->SetDriveOutputIPS(m_leftOutput, m_rightOutput);
    DBStringPrintf(DBStringPos::DB_LINE4, "vAr l=%1.2lf r=%1.2lf", m_leftOutput,
                   m_rightOutput);
    // printf("velocity l=%1.2lf r=%1.2lf\n", m_leftOutput, m_rightOutput);
    DBStringPrintf(DB_LINE3, "lsp %.0f  a %.0f\n", m_leftOutput,
                   state->GetLeftRate());
}

void VelocityArcadeDriveController::SetJoysticks(double throttle, double turn) {
    throttle = Util::bound(throttle, -1.0, 1.0);
    turn = Util::bound(turn, -1.0, 1.0);

    m_leftOutput =
        (throttle - turn) * 144.0;  // 37522 = full throttle ticks per second
    m_rightOutput = (throttle + turn) * 144.0;
    // printf("left %lf  right %lf\n", m_leftOutput, m_rightOutput);
}
}
