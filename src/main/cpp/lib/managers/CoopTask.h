/*
 * CoopTask.h
 *
 *  Created on: Sep 5, 2015
 *      Author: Andrew
 *
 * Interface for objects that can be run by a TaskMgr.  Similar to 254's
 * Loopable interface.
 *
 * TaskMgr keeps a collection of these objects and runs them repeatedly when
 * appropriate.  In general, TaskPrePeriodic should be for tasks that read
 * sensors or calculate values that other tasks may need to read, TaskPeriodic
 * should be used for most things, and TaskPostPeriodic should be used for
 * tasks that might conflict with other Periodic tasks.  A task may register
 * any combination of hooks.
 */

#pragma once

#include "lib/util/Util.h"

namespace frc973 {

class TaskMgr;

/**
 * Interface for a CoopTask.
 */
class CoopTask {
public:
    /**
     * Construct a CoopTask.
     */
    CoopTask();
    virtual ~CoopTask();

    /**
     * Beginning of a robot mode.
     * @param mode The current operating mode of the robot.
     */
    virtual void TaskStartMode(RobotMode mode) {
    }

    /**
     * End of a robot mode.
     * @param mode The current operating mode of the robot.
     */
    virtual void TaskStopMode(RobotMode mode) {
    }

    /**
     * Runs before periodic.
     * @param mode The current operating mode of the robot.
     */
    virtual void TaskPrePeriodic(RobotMode mode) {
    }

    /**
     * Main periodic loop.
     * @param mode The current operating mode of the robot.
     */
    virtual void TaskPeriodic(RobotMode mode) {
    }

    /**
     * Runs after periodic.
     * @param mode The current operating mode of the robot.
     */
    virtual void TaskPostPeriodic(RobotMode mode) {
    }
};
}
