/*
 * PoofsJoystickHelper.h
 *
 *  Created on: 9/13/18
 *      Author: Kyle, Chris Mc
 */

#pragma once

#include "lib/logging/LogSpreadsheet.h"
#include "lib/managers/CoopTask.h"

namespace frc973 {

/**
 * Button mapping for the Poofs joystick.
 */
namespace PoofsJoysticks {
const unsigned int LeftXAxis = 1;  /**< Left X Axis ID. */
const unsigned int LeftYAxis = 0;  /**< Left Y Axis ID. */
const unsigned int RightXAxis = 3; /**< Right X Axis ID. */
const unsigned int RightYAxis = 2; /**< Right Y Axis ID. */

const unsigned int LeftTrigger = 1;  /**< Left Trigger ID. */
const unsigned int LeftBumper = 2;   /**< Left Bumper ID. */
const unsigned int RightTrigger = 3; /**< Left Trigger ID. */
const unsigned int RightBumper = 4;  /**< Left Bumper ID. */
}

/**
 * An observer for a Poofs Joystick.
 */
class PoofsJoystickObserver {
public:
    /**
     * Construct a Poofs Joystick observer.
     */
    PoofsJoystickObserver() {
    }
    virtual ~PoofsJoystickObserver() {
    }

    /**
     * This function is overriden by the subclass to handle a joystick button
     * event notification.
     * @param port The joystick port.
     * @param button The joystick button.
     * @param newState If true, specifies the button has been pressed, if false,
     * specifies the button has been released.
     */
    virtual void ObservePoofsJoystickStateChange(uint32_t port, uint32_t button,
                                                 bool newState) = 0;
};

/**
 * An observable Poofs Joystick.
 */
class ObservablePoofsJoystick
        : public CoopTask
        , public Joystick {
public:
    /**
     * Create an instance of the ObservablePoofsJoystick object. Requires the
     * information to instantiate the underlying WPI-Joystick, as well as
     * references to the scheduler that will run it and the observer that
     * will observe its state.
     * @param port The joystick port.
     * @param observer The JoystickObserver object for button event notification
     * callback.
     * @param scheduler The task manager this task will run on.
     * @param ds The driver station.
     */
    ObservablePoofsJoystick(uint16_t port, PoofsJoystickObserver *observer,
                            TaskMgr *scheduler, DriverStation *ds = nullptr);
    ~ObservablePoofsJoystick();

    /**
     * Register this joystick with a logger so that button state can be logged
     * every time the periodic funciton is called. Only registers with the
     * first call.
     * @param logger The spreadsheet to log to.
     */
    ObservablePoofsJoystick *RegisterLog(LogSpreadsheet *logger);

    /**
     * Get the value of the given axis with deadband.
     * @param axis Specifies the axis to get the value of.
     * @param fSquared Specifies whether the joystick input should be squared.
     * @param threshold Specifies the deadband threshold.
     */
    float GetRawAxisWithDeadband(int axis, bool fSquared = false,
                                 double threshold = DEADBAND_INPUT_THRESHOLD);

    /**
     * Left trigger button.
     * @return Whether the left trigger button is pressed.
     */
    bool GetLTrigger();

    /**
     * Left bumper button.
     * @return Whether the left bumper button is pressed.
     */
    bool GetLBumper();

    /**
     * Right trigger button.
     * @return Whether the right trigger button is pressed.
     */
    bool GetRTrigger();

    /**
     * Right bumper button.
     * @return Whether the right bumper button is pressed.
     */
    bool GetRBumper();

    /**
     * Get a bitstring containing the state of *all* buttons on the joystick.
     * Including any 'virtual' buttons like the 'joystick buttons'.
     * @return The bitstring of all buttons.
     */
    uint32_t GetAllButtons();

    /**
     * This function is called by the TaskMgr to check and process Joystick
     * button events.
     * @param mode The current operating mode of the robot.
     */
    void TaskPrePeriodic(RobotMode mode) override;

    static constexpr double DEADBAND_INPUT_THRESHOLD =
        0.05; /**< The deadband threshold on the joysticks. */
    static constexpr double VIRTUAL_JOYSTICK_THRESHOLD =
        0.5; /**< The virtual joystick threshold. */
protected:
    uint32_t m_port; /**< The port the joystick is plugged into. */

    /* For observer notification */
    PoofsJoystickObserver *m_observer; /**< The class to notify whenever a
                         change in the joystick occurs. */
    DriverStation *m_ds;               /**< The DriverStation operating on. */
    uint32_t m_prevBtn;                /**< The previous button. */
    TaskMgr *m_scheduler;              /**< The task manager object. */
    LogCell *m_logCell;                /**< The logger. */
};
}
