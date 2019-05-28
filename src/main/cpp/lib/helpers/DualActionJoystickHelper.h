/*
 * DualActionJoystickHelper.h
 *
 *  Created on: 9/18/18
 *      Author: Andrew
 */

#pragma once

#include "lib/logging/LogSpreadsheet.h"
#include "lib/managers/CoopTask.h"

namespace frc973 {

/**
 * Button mapping for the DualAction joystick.
 */
namespace DualAction {

const unsigned int BtnX = 1;         /**< Button X ID. */
const unsigned int BtnA = 2;         /**< Button A ID. */
const unsigned int BtnB = 3;         /**< Button B ID. */
const unsigned int BtnY = 4;         /**< Button Y ID. */
const unsigned int LeftBumper = 5;   /**< Left Bumper ID. */
const unsigned int RightBumper = 6;  /**< Right Bumper ID. */
const unsigned int LeftTrigger = 7;  /**< Left Trigger ID. */
const unsigned int RightTrigger = 8; /**< Right Trigger ID. */
const unsigned int Back = 9;         /**< Back Button ID. */
const unsigned int Start = 10;       /**< Start Button ID. */

const unsigned int LJoystickBtn = 11; /**< Left Joystick Button ID. */
const unsigned int RJoystickBtn = 12; /**< Right Joystick Button ID. */

const unsigned int DPadUpVirtBtn = 22;    /**< DPad Up Virtual Button ID. */
const unsigned int DPadDownVirtBtn = 23;  /**< DPad Down Virtual Button ID. */
const unsigned int DPadLeftVirtBtn = 24;  /**< DPad Left Virtual Button ID. */
const unsigned int DPadRightVirtBtn = 25; /**< DPad Right Virtual Button ID. */

const unsigned int LXAxisVirtButton = 26; /**< Left X Axis Virtual Button ID. */
const unsigned int LYAxisVirtButton = 27; /**< Left Y Axis Virtual Button ID. */
const unsigned int RXAxisVirtButton = 28; /**< Right X Axis Virt Button ID. */
const unsigned int RYAxisVirtButton = 29; /**< Right Y Axis Virt Button ID. */
const unsigned int DXAxisVirtButton = 30; /**< DPad X Axis Virtual Button ID. */
const unsigned int DYAxisVirtButton = 31; /**< DPad Y Axis Virtual Button ID. */

const unsigned int LeftXAxis = 0;  /**< Left X Axis ID. */
const unsigned int LeftYAxis = 1;  /**< Left Y Axis ID. */
const unsigned int RightXAxis = 2; /**< Right X Axis ID. */
const unsigned int RightYAxis = 3; /**< Right Y Axis ID. */
const unsigned int DPadXAxis = 4;  /**< DPad X Axis ID. */
const unsigned int DPadYAxis = 5;  /**< DPad Y Axis ID. */
}

/**
 * An observer for a DualAction Joystick.
 */
class DualActionJoystickObserver {
public:
    /**
     * Construct a DualActionJoystickObserver.
     */
    DualActionJoystickObserver() {
    }
    virtual ~DualActionJoystickObserver() {
    }

    /**
     * This function is overriden by the subclass to handle a joystick button
     * event notification.
     * @param port The joystick port.
     * @param button The joystick button.
     * @param newState If true, specifies the button has been pressed, if false,
     * specifies the button has been released.
     */
    virtual void ObserveDualActionJoystickStateChange(uint32_t port,
                                                      uint32_t button,
                                                      bool newState) = 0;
};

/**
 * This class observes a given joystick and notifies the given callback
 * on any joystick event. This is particularly useful for catching the
 * *edge* of a button press or release event. Also lets you use a joystick
 * axis as a button in an easy way.
 */
class ObservableDualActionJoystick
        : public CoopTask
        , public Joystick {
public:
    /**
     * Create an instance of the ObservableDualActionJoystick object. Requires
     * the information to instantiate the underlying WPI-Joystick, as well as
     * references to the scheduler that will run it and the observer that
     * will observe its state.
     * @param port The joystick's port.
     * @param observer The DualActionJoystickObserver object for button event
     * notification callback.
     * @param scheduler The TaskMgr this task will run on.
     * @param ds The DriverStation.
     */
    ObservableDualActionJoystick(uint16_t port,
                                 DualActionJoystickObserver *observer,
                                 TaskMgr *scheduler,
                                 DriverStation *ds = nullptr);
    ~ObservableDualActionJoystick();

    /**
     * Register this joystick with a logger so that button state can be logged
     * every time the periodic funciton is called. Only registers with the
     * first call.
     * @param logger The LogSpreadsheet to log to.
     */
    ObservableDualActionJoystick *RegisterLog(LogSpreadsheet *logger);

    /**
     * Get the value of the given axis with deadband.
     * @param axis The axis to get the value of.
     * @param fSquared The joystick input should be squared.
     * @param threshold The deadband threshold.
     * @return The raw axis value with deadband.
     */
    float GetRawAxisWithDeadband(int axis, bool fSquared = false,
                                 double threshold = DEADBAND_INPUT_THRESHOLD);

    /**
     * DPad Up Virtual button.
     * @return Whether the DPad's up button is pressed.
     */
    bool GetDPadUpVirtButton();

    /**
     * DPad DownVirtual button.
     * @return Whether the DPad's down button is pressed.
     */
    bool GetDPadDownVirtButton();

    /**
     * DPad Left Virtual button.
     * @return Whether the DPad's left button is pressed.
     */
    bool GetDPadLeftVirtButton();

    /**
     * DPad Right Virtual button.
     * @return Whether the DPad's right button is pressed.
     */
    bool GetDPadRightVirtButton();

    /**
     * Pretend the Left X Axis is a button. By default it is not pressed.
     * If the user pushes it mostly forward (say, more than half way), say
     * that button is pressed. If the user pulls it mostly backwards (say,
     * more than half way), say that button is released. If it's anywhere
     * in between, rememember what it last was.
     */

    /**
     * Left X Virtual button.
     * @return Whether the left X virtual button is pressed.
     */
    bool GetLXVirtButton();

    /**
     * Left Y Virtual button.
     * @return Whether the left Y virtual button is pressed.
     */
    bool GetLYVirtButton();

    /**
     * Right X Virtual button.
     * @return Whether the right X virtual button is pressed.
     */
    bool GetRXVirtButton();

    /**
     * Right Y Virtual button.
     * @return Whether the right Y virtual button is pressed.
     */
    bool GetRYVirtButton();

    /**
     * DPad X Virtual button.
     * @return Whether the DPad X virtual button is pressed.
     */
    bool GetDXVirtButton();

    /**
     * DPad Y Virtual button.
     * @return Whether the DPad Y virtual button is pressed.
     */
    bool GetDYVirtButton();

    /**
     * Get a bitstring containing the state of *all* buttons on the joystick.
     * Including any 'virtual' buttons like the 'joystick buttons'.
     * @return The bitstring of all buttons.
     */
    uint32_t GetAllButtons();

    /**
     * This function is called by the TaskMgr to check and process Joystick
     * button events.
     * @param mode The current RobotMode.
     */
    void TaskPrePeriodic(RobotMode mode) override;

    static constexpr double DEADBAND_INPUT_THRESHOLD =
        0.05; /**< The deadband threshold on the joysticks. */
    static constexpr double VIRTUAL_JOYSTICK_THRESHOLD =
        0.5; /**< The virtual joystick threshold. */
protected:
    uint32_t m_port; /**< The port the joystick is plugged into. */

    /* For observer notification */
    DualActionJoystickObserver *m_observer; /**< The DualActionJoystickObserver
                          to notify whenever a change in the joystick occurs. */
    DriverStation *m_ds;  /**< The DriverStation operating on. */
    uint32_t m_prevBtn;   /**< The previous button. */
    TaskMgr *m_scheduler; /**< The TaskMgr object. */
    LogCell *m_logCell;   /**< The LogCell. */

    /* For remembering states of sticky buttons */
    bool m_lastLXVal; /**< The last left joystick's x axis value. */
    bool m_lastLYVal; /**< The last left joystick's y axis value. */
    bool m_lastRXVal; /**< The last right joystick's x axis value. */
    bool m_lastRYVal; /**< The last right joystick's y axis value. */
    bool m_lastDXVal; /**< The last DPad's x axis value. */
    bool m_lastDYVal; /**< The last DPad's y axis value. */
};
}
