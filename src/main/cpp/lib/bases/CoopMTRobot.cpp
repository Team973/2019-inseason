/*
 * CoopMTRobot.cpp
 *
 *  Created on: Sep 1, 2015
 *      Author: Andrew
 */

#include "lib/bases/CoopMTRobot.h"

static constexpr bool ENABLE_PROFILING = false;

namespace frc973 {

CoopMTRobot::CoopMTRobot(void)
        : TimedRobot()
        , TaskMgr()
        , m_prevMode(RobotMode::MODE_DISABLED)
        , m_robotModeMutex(PTHREAD_MUTEX_INITIALIZER) {
    printf("Going to construct frc::Scheduler\n");
    Scheduler::GetInstance();
    printf("Successfully constructed frc::Scheduler\n");
}

CoopMTRobot::~CoopMTRobot() {
}

void CoopMTRobot::RobotInit() {
    char hostName[MAXHOSTNAMELEN];
    gethostname(hostName, ARRAYSIZE(hostName));
    printf(ESC_PREFIX SGR_FG_BLACK ESC_SEP SGR_BG_WHITE ESC_SUFFIX
           "\n****************************************\n"
           "     Name: %s\n"
           "  Program: %s\n"
           " Compiled: %s, %s\n"
           "\n****************************************\n" ESC_NORMAL,
           hostName, PROGRAM_NAME, __DATE__, __TIME__);

    printf(
        "Laws of robotics:\n"
        "A Robot may not injure a human being or, through inaction, alow a "
        "human being to come to harm.\n"
        "A Robot must obey orders given to you by human beings, except where "
        "such orders would conflict with the first law.\n"
        "A Robot must protect its own existence, as long as such protection "
        "does not conflict with the first or second laws. \n");

    Initialize();
}

void CoopMTRobot::DisabledInit() {
    this->ModeStop(this->m_prevMode);
    pthread_mutex_lock(&m_robotModeMutex);
    this->m_prevMode = RobotMode::MODE_DISABLED;
    pthread_mutex_unlock(&m_robotModeMutex);
    this->ModeStart(this->m_prevMode);
}

void CoopMTRobot::AutonomousInit() {
    this->ModeStop(this->m_prevMode);
    pthread_mutex_lock(&m_robotModeMutex);
    this->m_prevMode = RobotMode::MODE_AUTO;
    pthread_mutex_unlock(&m_robotModeMutex);
    this->ModeStart(this->m_prevMode);
}

void CoopMTRobot::TeleopInit() {
    this->ModeStop(this->m_prevMode);
    pthread_mutex_lock(&m_robotModeMutex);
    this->m_prevMode = RobotMode::MODE_TELEOP;
    pthread_mutex_unlock(&m_robotModeMutex);
    this->ModeStart(this->m_prevMode);
}

void CoopMTRobot::TestInit() {
    this->ModeStop(this->m_prevMode);
    pthread_mutex_lock(&m_robotModeMutex);
    this->m_prevMode = RobotMode::MODE_TEST;
    pthread_mutex_unlock(&m_robotModeMutex);
    this->ModeStart(this->m_prevMode);
}

void CoopMTRobot::DisabledPeriodic() {
    uint64_t startTime = GetUsecTime();

    this->TaskPrePeriodicAll(this->m_prevMode);
    if (ENABLE_PROFILING) {
        printf("DisabledPrePeriodic toook %llu us\n",
               GetUsecTime() - startTime);
    }

    this->DisabledContinuous();
    if (ENABLE_PROFILING) {
        printf("DisabledContinuous toook %llu us\n", GetUsecTime() - startTime);
    }

    this->AllStateContinuous();
    if (ENABLE_PROFILING) {
        printf("DisabledAllStateContinuous toook %llu us\n",
               GetUsecTime() - startTime);
    }

    this->TaskPeriodicAll(this->m_prevMode);
    if (ENABLE_PROFILING) {
        printf("DisabledTaskPeriodic toook %llu us\n",
               GetUsecTime() - startTime);
    }

    this->TaskPostPeriodicAll(this->m_prevMode);
    if (ENABLE_PROFILING) {
        printf("DisabledTaskPostPeriodic toook %llu us\n",
               GetUsecTime() - startTime);
    }
}

void CoopMTRobot::AutonomousPeriodic() {
    this->TaskPrePeriodicAll(this->m_prevMode);
    this->AutonomousContinuous();
    this->AllStateContinuous();
    this->TaskPeriodicAll(this->m_prevMode);
    this->TaskPostPeriodicAll(this->m_prevMode);
}

void CoopMTRobot::TeleopPeriodic() {
    this->TaskPrePeriodicAll(this->m_prevMode);
    this->TeleopContinuous();
    this->AllStateContinuous();
    this->TaskPeriodicAll(this->m_prevMode);
    this->TaskPostPeriodicAll(this->m_prevMode);
}

void CoopMTRobot::TestPeriodic() {
    this->TaskPrePeriodicAll(this->m_prevMode);
    this->TestContinuous();
    this->AllStateContinuous();
    this->TaskPeriodicAll(this->m_prevMode);
    this->TaskPostPeriodicAll(this->m_prevMode);
}

void CoopMTRobot::ModeStop(RobotMode toStop) {
    switch (toStop) {
        case RobotMode::MODE_DISABLED:
            TaskStopModeAll(toStop);
            DisabledStop();
            break;
        case RobotMode::MODE_AUTO:
            TaskStopModeAll(toStop);
            AutonomousStop();
            break;
        case RobotMode::MODE_TELEOP:
            TaskStopModeAll(toStop);
            TeleopStop();
            break;
        case RobotMode::MODE_TEST:
            TaskStopModeAll(toStop);
            TestStop();
            break;
    }
}

void CoopMTRobot::ModeStart(RobotMode toStart) {
    switch (toStart) {
        case RobotMode::MODE_DISABLED:
            TaskStartModeAll(toStart);
            DisabledStart();
            break;
        case RobotMode::MODE_AUTO:
            TaskStartModeAll(toStart);
            AutonomousStart();
            break;
        case RobotMode::MODE_TELEOP:
            TaskStartModeAll(toStart);
            TeleopStart();
            break;
        case RobotMode::MODE_TEST:
            TaskStartModeAll(toStart);
            TestStart();
            break;
    }
}

bool CoopMTRobot::IsDisabled() const {
    pthread_mutex_lock(&m_robotModeMutex);
    bool res = m_prevMode == MODE_DISABLED;
    pthread_mutex_unlock(&m_robotModeMutex);
    return res;
}

bool CoopMTRobot::IsEnabled() const {
    return !IsDisabled();
}

bool CoopMTRobot::IsOperatorControl() const {
    pthread_mutex_lock(&m_robotModeMutex);
    bool res = m_prevMode == MODE_TELEOP;
    pthread_mutex_unlock(&m_robotModeMutex);
    return res;
}

bool CoopMTRobot::IsAutonomous() const {
    pthread_mutex_lock(&m_robotModeMutex);
    bool res = m_prevMode == MODE_AUTO;
    pthread_mutex_unlock(&m_robotModeMutex);
    return res;
}

bool CoopMTRobot::IsTest() const {
    pthread_mutex_lock(&m_robotModeMutex);
    bool res = m_prevMode == MODE_TEST;
    pthread_mutex_unlock(&m_robotModeMutex);
    return res;
}
}
