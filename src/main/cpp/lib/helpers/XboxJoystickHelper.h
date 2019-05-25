/*
 * XboxJoystickHelper.h
 *
 *  Created on: Sep 13, 2018
 *      Author: Kyle
 */

#pragma once

#include "frc/WPILib.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/managers/CoopTask.h"
#include <stdint.h>

using namespace frc;

namespace frc973 {

namespace Xbox {
const unsigned int BtnX = 3;        /**< X Button */
const unsigned int BtnA = 1;        /**< A Button */
const unsigned int BtnB = 2;        /**< B Button */
const unsigned int BtnY = 4;        /**< Y Button */
const unsigned int LeftBumper = 5;  /**< Left Bumper Button */
const unsigned int RightBumper = 6; /**< Right Bumper Button */
const unsigned int Back = 7;        /**< Back Button */
const unsigned int Start = 8;       /**< Start Button */

const unsigned int LJoystickBtn = 9;  /**< Left Joystick Button */
const unsigned int RJoystickBtn = 10; /**< Right Joystick Button */

const unsigned int DPadUpVirtBtn = 11;    /**< DPad Up Button*/
const unsigned int DPadDownVirtBtn = 12;  /**< DPad Down Button*/
const unsigned int DPadLeftVirtBtn = 13;  /**< DPad Left Button*/
const unsigned int DPadRightVirtBtn = 14; /**< DPad Right Button*/

const unsigned int LeftXAxis = 0;        /**< Left Joystick X Axis*/
const unsigned int LeftYAxis = 1;        /**< Left Joystick Y Axis*/
const unsigned int RightXAxis = 4;       /**< Right Joystick X Axis*/
const unsigned int RightYAxis = 5;       /**< Right Joystick Y Axis*/
const unsigned int LeftTriggerAxis = 2;  /**< Left Trigger Button Axis*/
const unsigned int RightTriggerAxis = 3; /**< Right Trigger Button Axis*/
}  // namespace Xbox

/**
 * An observer for a Xbox Joystick.
 */
class XboxJoystickObserver {
public:
    XboxJoystickObserver() {
    }
    virtual ~XboxJoystickObserver() {
    }
    /**
     * This function is overriden by the subclass to handle an Xbox button
     * event notification.
     * @param port The joystick port.
     * @param button The joystick button.
     * @param newState If true, specifies the button has been pressed, if false,
     * specifies the button has been released.
     */
    virtual void ObserveXboxJoystickStateChange(uint32_t port, uint32_t button,
                                                bool newState) = 0;
};

/**
 * This class observes a given joystick and notifies the given callback
 * on any joystick event. This is particularly useful for catching the
 * *edge* of a button press or release event. Also lets you use a joystick
 * axis as a button in an easy way.
 */
class ObservableXboxJoystick
        : public CoopTask
        , public XboxController {
public:
    /**
     * Create an instance of the ObservableXboxJoystickJoystick object. Requires
     * the information to instantiate the underlying WPI-Joystick, as well as
     * references to the scheduler that will run it and the observer that
     * will observe its state.
     * @param port The joystick port.
     * @param observer The JoystickObserver object for button event notification
     * callback.
     * @param scheduler The task manager this task will run on.
     * @param ds The driver station.
     */
    ObservableXboxJoystick(uint16_t port, XboxJoystickObserver *observer,
                           TaskMgr *scheduler, DriverStation *ds = nullptr);
    ~ObservableXboxJoystick();

    /**
     * Register this joystick with a logger so that button state can be logged
     * every time the periodic funciton is called. Only registers with the
     * first call.
     * @param logger The spreadsheet to log to.
     */
    ObservableXboxJoystick *RegisterLog(LogSpreadsheet *logger);

    /**
     * Get the value of the given axis with deadband.
     * @param axis Specifies the axis to get the value of.
     * @param fSquared Specifies whether the joystick input should be squared.
     * @param threshold Specifies the deadband threshold.
     */
    float GetRawAxisWithDeadband(int axis, bool fSquared = false,
                                 double threshold = DEADBAND_INPUT_THRESHOLD);

    bool GetDPadUpVirtButton(); /**< Check whether the up button on the d pad is
                       pressed. */
    bool GetDPadDownVirtButton();  /**< Check whether the down button on the d
                          pad is pressed. */
    bool GetDPadLeftVirtButton();  /**< Check whether the left button on the d
                          pad is pressed. */
    bool GetDPadRightVirtButton(); /**< Check whether the right button on the d
                          pad is pressed. */

    /**
     * Pretend the Left X Axis is a button.  By default it is not pressed.
     * If the user pushes it mostly forward (say, more than half way), say
     * that button is pressed.  If the user pulls it mostly backwards (say,
     * more than half way), say that button is released.  If it's anywhere
     * in between, rememember what it last was.
     */

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
    XboxJoystickObserver *m_observer; /**< The class to notify whenever a change
                         in the joystick occurs. */
    DriverStation *m_ds;              /**< The DriverStation operating on.*/
    uint32_t m_prevBtn;               /**< The previous button.*/
    TaskMgr *m_scheduler;             /**< The task manager object.*/
    LogCell *m_logCell;               /**< The logger.*/
};
}
