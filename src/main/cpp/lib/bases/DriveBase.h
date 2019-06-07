/*
 * DriveBase.h
 *
 *  Created on: Oct 29, 2015
 *      Author: Andrew
 */

#pragma once

#include "lib/managers/CoopTask.h"
#include "lib/managers/TaskMgr.h"

namespace frc973 {

/**
 * Interface for a class that determines the current state of the drivetrain.
 */
class DriveStateProvider {
public:
    /**
     * Construct a DriveStateProvider.
     */
    DriveStateProvider() {
    }
    virtual ~DriveStateProvider() {
    }

    /**
     * Gets the current angle in degrees.
     * @return The current angle.
     */
    virtual double GetAngle() const = 0;

    /**
     * Gets the current angular rate in degrees/second.
     * @return The current angular rate.
     */
    virtual double GetAngularRate() const = 0;

    /**
     * Gets the current left distance in inches.
     * @return The current left distance.
     */
    virtual double GetLeftDist() const = 0;

    /**
     * Gets the current right distance in inches.
     * @return The current right distance.
     */
    virtual double GetRightDist() const = 0;

    /**
     * Gets the current left velocity in inches/second.
     * @return The current left velocity.
     */
    virtual double GetLeftRate() const = 0;

    /**
     * Gets the current right velocity in inches/second.
     * @return The current right velocity.
     */
    virtual double GetRightRate() const = 0;

    /**
     * Gets the current average distance in inches.
     * @return The current average distance.
     */
    virtual double GetDist() const = 0;

    /**
     * Gets the current average velocity in inches/second.
     * @return The current average velocity.
     */
    virtual double GetRate() const = 0;
};

/**
 * Interface for a class that can take drive output.
 */
class DriveControlSignalReceiver {
public:
    /**
     * A drive control signal receiver.
     */
    DriveControlSignalReceiver() {
    }
    virtual ~DriveControlSignalReceiver() {
    }

    /**
     * Receive calculated motor powers from a controller.
     * Should only be called from a child of DriveController.
     */

    /**
     * Create a setpoint velocity in inches/second.
     * @param left Left setpoint.
     * @param right Right setpoint.
     */
    virtual void SetDriveOutputIPS(double left, double right) = 0;

    /**
     * Create a setpoint position in inches.
     * @param left Left setpoint.
     * @param right Right setpoint.
     */
    virtual void SetDriveOutputPosInches(double left, double right) = 0;

    /**
     * Create a setpoint voltage.
     * @param left Left setpoint.
     * @param right Right setpoint.
     */
    virtual void SetDriveOutputVBus(double left, double right) = 0;

    /**
     * Set the current limit in amperes.
     * @param limit The desired current limit.
     */
    virtual void ConfigDriveCurrentLimit(double limit) = 0;

    /**
     * Disable the current limiting on the drivetrain.
     */
    virtual void DisableDriveCurrentLimit() = 0;
};

/**
 * Interface that uses a DriveStateProvider, calculates output, and sends it to
 * DriveControlSignalReceiver. This is the base for all drive controllers.
 */
class DriveController {
public:
    /**
     * A drive controller.
     */
    DriveController() {
    }
    virtual ~DriveController() {
    }

    /**
     * Use the input signals from the |state| and calculate some output, then
     * send that output to the |out|.
     * @param state The DriveStateProvider for handling incoming messages.
     * @param out The DriveControlSignalReceiver for handling outgoing messages.
     */
    virtual void CalcDriveOutput(DriveStateProvider *state,
                                 DriveControlSignalReceiver *out) = 0;

    /**
     * Checks with the current controller to see if we are on target. If there
     * is no controller currently selected, just return false.
     * @return Whether the current controller things are done.
     */
    virtual bool OnTarget() = 0;

    /**
     * Called when the drive controller is set active by the Drive subsystem.
     * @param out The DriveControlSignalReceiver for handling outgoing messages.
     */
    virtual void Start(DriveControlSignalReceiver *out) {
    }

    /**
     * Called when the drive controller is set inactive by the Drive subsystem.
     * @param out The DriveControlSignalReceiver for handling outgoing messages.
     */
    virtual void Stop(DriveControlSignalReceiver *out) {
    }
};

/**
 * Base class for a robot drive. DriveBase keeps track of one DriveController
 * and uses it to calculate drive output, then drives the motors with those
 * calculated values. CoopTask handles calling TaskPostPeriodic once a cycle.
 */
class DriveBase : public CoopTask {
public:
    /**
     * Creates a new DriveBase Object. The DriveBase object stores a drive
     * controller (an object capable of calculating motor outputs) and uses it
     * to calculate drive outputs, then drive those drive outputs.
     * @param scheduler
     * @param out The DriveControlSignalReceiver for handling outgoing messages.
     * @param state The DriveStateProvider for handling incoming messages.
     * @param controller The drive controller for handling movements.
     */
    DriveBase(TaskMgr *scheduler, DriveControlSignalReceiver *out,
              DriveStateProvider *state, DriveController *controller = nullptr);
    virtual ~DriveBase();

    /**
     * When making calls to PID-like-commands, parameters can be RelativeTo the
     * world (Absolute), RelativeTo the current position (Now), or RelativeTo
     * the current setpoint (SetPoint).
     */
    enum RelativeTo
    {
        Absolute, /**< Relative to where the robot was turned on. */
        Now,      /**< Relative to the current position. */
        SetPoint  /**< Relative to the current setpoint. */
    };

    /**
     * Get input from the currently active DriveController. This method comes
     * from CoopTask and is called automatically once a cycle by m_scheduler.
     * @param mode The current RobotMode.
     */
    void TaskPostPeriodic(RobotMode mode) override;

    /**
     * Change the DriveController currently active.
     * @param controller The drive controller for hangling movements.
     */
    void SetDriveController(DriveController *controller);

    /**
     * Checks with the current controller to see if we are on target. If there
     * is no controller currently selected, just return false.
     * @return Whether the current controller things are done.
     */
    bool OnTarget();

protected:
    /**
     * Task manager member.
     */
    TaskMgr *m_scheduler;

    /**
     * Signal receiver member.
     */
    DriveControlSignalReceiver *m_driveOutput;

private:
    DriveStateProvider *m_stateProvider;
    DriveController *m_controller;
};
}
