/*
 * ControllerBase.h
 *
 * Defines a few related classes for creating control systems in an object
 * oriented manor.  Yes, we could merge this with the controller classes
 * in DriveBase using templates, but the frc toolchain doesn't have fantastic
 * support for templates (and we're trying to keep compile time down :p).
 *
 * Am I really solving a problem here?  Let's just try the old-style of
 * organizing controls and see if it gets messy.
 *
 *  Created on: Jan 28, 2016
 *      Author: andrew
 */

#pragma once

#include "lib/managers/CoopTask.h"
#include "lib/managers/TaskMgr.h"

using namespace frc;

namespace frc973 {

class SimpleControlSystem;

/**
 * Interface for something that can tell the current state of the system. Used
 * by the controller to get its input signal.
 */
class SimpleControlStateProvider {
    /**
     * A simple control state provider.
     */
    SimpleControlStateProvider() {
    }
    virtual ~SimpleControlStateProvider() {
    }

    /**
     * Return the velocity of a system.
     * @param system The control system.
     * @return The velocity of the system.
     */
    virtual double GetRate(SimpleControlSystem *system = nullptr) = 0;

    /**
     * Return the position of a system.
     * @param system The control system.
     * @return The position of the system.
     */
    virtual double GetPosition(SimpleControlSystem *system = nullptr) = 0;
};

/**
 * Interface for something that can send signal to the system (power motors).
 * The controller sends its output signal here
 */
class SimpleControlSignalReceiver {
    /**
     * A simple control signal receiver.
     */
    SimpleControlSignalReceiver() {
    }
    virtual ~SimpleControlSignalReceiver() {
    }

    /**
     * Receive calculated motor powers from a controller.
     * Should only be called from a child of SimpleController.
     * @param output Output to motors.
     * @param system The control system.
     */
    virtual void SetControllerOutput(double output,
                                     SimpleControlSystem *system = nullptr) = 0;
};

/**
 * Interface for a simple controller. One that receives system state from a
 * SimpleControlStateProvider and sends output to a SimpleControlSignalReceiver.
 * This could be generalized with templates, but this is easier to debug.
 */
class SimpleController {
public:
    /**
     * A SimpleController for driving.
     * @param system The control system.
     */
    SimpleController(SimpleControlSystem *system);
    virtual ~SimpleController();

    /**
     * Enable the controller.
     */
    virtual void Enable();

    /**
     * Disable the controller.
     */
    virtual void Disable();

    /**
     * Use the input signals from |angle| and |dist| and calculate some output,
     * then send that output to |out|.
     * @param state The state provider for handling incoming messages.
     * @param out Signal receiver for handling outgoing messages.
     */
    virtual void CalcControllerOutput(SimpleControlStateProvider *state,
                                      SimpleControlSignalReceiver *out) = 0;

    /**
     * Checks with the current controller to see if we are on target. If there
     * is no controller currently selected, just return false.
     * @return Whether the current controller things are done.
     */
    virtual bool OnTarget() = 0;
};

/**
 * Interface for a simple control system that combines other simple controllers.
 */
class SimpleControlSystem : public CoopTask {
public:
    /**
     * A SimpleControlSystem for combining the controller, signal receiver, and
     * state provider.
     * @param scheduler The main task manager.
     * @param state The state provider for handling incoming messages.
     * @param out The signal receiver for handling outgoing messages.
     * @param controller The drive controller for hangling movements.
     */
    SimpleControlSystem(TaskMgr *scheduler, SimpleControlStateProvider *state,
                        SimpleControlSignalReceiver *out,
                        SimpleController *controller = nullptr)
            : m_scheduler(scheduler)
            , m_state(state)
            , m_out(out)
            , m_activeController(controller) {
        m_scheduler->RegisterTask("Simple Control System", this,
                                  TASK_POST_PERIODIC);
    }
    virtual ~SimpleControlSystem() {
        m_scheduler->UnregisterTask(this);
    }

    /**
     * Set the active controller.
     * @param controller The drive controller for hangling movements.
     */
    void SetActiveController(SimpleController *controller) {
        if (m_activeController) {
            m_activeController->Disable();
        }
        m_activeController = controller;
        m_activeController->Enable();
    }

    /**
     * Checks with the current controller to see if we are on target. If there
     * is no controller currently selected, just return false.
     * @return Whether the current controller things are done.
     */
    bool OnTarget() {
        if (!m_activeController) {
            return false;
        }
        return m_activeController->OnTarget();
    }

    /**
     * Periodically recalculate drive outputs based on controller's drive
     * method.
     * @param mode The current operating mode of the robot.
     */
    void TaskPostPeriodic(RobotMode mode) override {
        if (m_activeController != nullptr) {
            m_activeController->CalcControllerOutput(m_state, m_out);
        }
    }

private:
    TaskMgr *m_scheduler;
    SimpleControlStateProvider *m_state;
    SimpleControlSignalReceiver *m_out;
    SimpleController *m_activeController;
};
}
