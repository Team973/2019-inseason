/*
 * DriveBase.cpp
 *
 *  Created on: Oct 29, 2015
 *      Author: Andrew
 */

#include "lib/bases/DriveBase.h"

namespace frc973 {

DriveBase::DriveBase(TaskMgr *scheduler, DriveControlSignalReceiver *out,
                     DriveStateProvider *state, DriveController *controller)
        : m_scheduler(scheduler)
        , m_driveOutput(out)
        , m_stateProvider(state)
        , m_controller(controller) {
    m_scheduler->RegisterTask("DriveBase", this, TASK_POST_PERIODIC);
}

DriveBase::~DriveBase() {
    m_scheduler->UnregisterTask(this);
}

void DriveBase::TaskPostPeriodic(RobotMode mode) {
    if (m_controller != nullptr) {
        m_controller->CalcDriveOutput(m_stateProvider, m_driveOutput);
    }
}

void DriveBase::SetDriveController(DriveController *newController) {
    DriveController *oldController = m_controller;

    if (m_controller != nullptr && newController != oldController) {
        m_controller->Stop(m_driveOutput);
    }

    m_controller = newController;

    if (m_controller != nullptr && newController != oldController) {
        m_controller->Start(m_driveOutput);
    }
}

bool DriveBase::OnTarget() {
    if (m_controller) {
        return m_controller->OnTarget();
    }
    else {
        return false;
    }
}
}
