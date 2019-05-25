/*
 * TaskMgr.cpp
 *
 *  Created on: Sep 5, 2015
 *      Author: Andrew
 */

#include "lib/managers/TaskMgr.h"

static constexpr bool ENABLE_PROFILING = false;

namespace frc973 {

TaskMgr::TaskMgr(void) : m_numTasks(0) {
}

TaskMgr::~TaskMgr() {
}

bool TaskMgr::RegisterTask(const char *taskName, CoopTask *task,
                           uint32_t flags) {
    bool success = false;
    int index;

    index = this->FindTask(task);
    if (index != -1) {
        // If the task is already registered, just add to the existing flags
        m_taskFlags[index] |= flags;
        success = true;
    }
    else if (m_numTasks < MAX_NUM_TASKS) {
        strncpy(m_taskNames[m_numTasks], taskName, MAX_TASK_NAME_LEN);
        m_tasks[m_numTasks] = task;
        m_taskFlags[m_numTasks] = flags;
        m_numTasks++;
        success = true;
    }

    fprintf(stderr, "Task %s registered to %p with result %d (1 is go0d)\n",
            taskName, (void *)this, success);

    return success;
}

bool TaskMgr::UnregisterTask(CoopTask *task) {
    bool success = false;
    int taskIndex, j;

    taskIndex = this->FindTask(task);
    if (taskIndex != -1) {
        // Shift tasks to replace the removed one
        for (j = taskIndex + 1; j < m_numTasks; j++) {
            strcpy(m_taskNames[j - 1], m_taskNames[j]);
            m_tasks[j - 1] = m_tasks[j];
            m_taskFlags[j - 1] = m_taskFlags[j];
        }

        // clear out the last task in the list
        m_numTasks--;
        m_taskNames[m_numTasks][0] = '\0';
        m_tasks[m_numTasks] = NULL;
        m_taskFlags[m_numTasks] = 0;

        success = true;
    }

    return success;
}

void TaskMgr::TaskStartModeAll(RobotMode mode) {
    uint64_t startTime, endTime;

    for (int i = 0; i < m_numTasks; i++) {
        if (m_taskFlags[i] & TASK_START_MODE) {
            startTime = GetUsecTime();
            m_tasks[i]->TaskStartMode(mode);
            endTime = GetUsecTime();

            if (ENABLE_PROFILING) {
                printf("TaskStartMode(%d) for %s took %llu us\n", mode,
                       m_taskNames[i], endTime - startTime);
            }
        }
    }
}

void TaskMgr::TaskStopModeAll(RobotMode mode) {
    uint64_t startTime, endTime;

    // stop tasks in the reverse order they were started in
    for (int i = m_numTasks - 1; i >= 0; i--) {
        if (m_taskFlags[i] & TASK_STOP_MODE) {
            startTime = GetUsecTime();
            m_tasks[i]->TaskStopMode(mode);
            endTime = GetUsecTime();

            if (ENABLE_PROFILING) {
                printf("TaskStopMode(%d) for %s took %llu us\n", mode,
                       m_taskNames[i], endTime - startTime);
            }
        }
    }
}

void TaskMgr::TaskPrePeriodicAll(RobotMode mode) {
    uint64_t startTime, endTime;

    for (int i = 0; i < m_numTasks; i++) {
        if (m_taskFlags[i] & TASK_PRE_PERIODIC) {
            startTime = GetUsecTime();
            m_tasks[i]->TaskPrePeriodic(mode);
            endTime = GetUsecTime();

            if (ENABLE_PROFILING) {
                printf("TaskPrePeriodicAll(%d) for %s took %llu us\n", mode,
                       m_taskNames[i], endTime - startTime);
            }
        }
    }
}

void TaskMgr::TaskPeriodicAll(RobotMode mode) {
    uint64_t startTime, endTime;

    for (int i = 0; i < m_numTasks; i++) {
        if (m_taskFlags[i] & TASK_PERIODIC) {
            startTime = GetUsecTime();
            m_tasks[i]->TaskPeriodic(mode);
            endTime = GetUsecTime();

            if (ENABLE_PROFILING) {
                printf("TaskPeriodicAll(%d) for %s took %llu us\n", mode,
                       m_taskNames[i], endTime - startTime);
            }
        }
    }
}

void TaskMgr::TaskPostPeriodicAll(RobotMode mode) {
    uint64_t startTime, endTime;

    for (int i = 0; i < m_numTasks; i++) {
        if (m_taskFlags[i] & TASK_POST_PERIODIC) {
            startTime = GetUsecTime();
            m_tasks[i]->TaskPostPeriodic(mode);
            endTime = GetUsecTime();

            if (ENABLE_PROFILING) {
                printf("TaskPostPeriodicAll(%d) for %s took %llu us\n", mode,
                       m_taskNames[i], endTime - startTime);
            }
        }
    }
}

int TaskMgr::FindTask(CoopTask *task) {
    int index = -1;

    for (int i = 0; i < m_numTasks; i++) {
        if (m_tasks[i] == task) {
            index = i;
            break;
        }
    }

    return index;
}
}
