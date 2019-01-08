/*
 * PoofsJoystickHelper.h
 *
 *  Created on: 9/13/18
 *      Author: Kyle, Chris Mc
 */

#pragma once

#include "frc/WPILib.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/managers/CoopTask.h"
#include <stdint.h>

using namespace frc;

namespace frc973 {

namespace PoofsJoysticks {
const unsigned int LeftXAxis = 1;
const unsigned int LeftYAxis = 0;
const unsigned int RightXAxis = 3;
const unsigned int RightYAxis = 2;

const unsigned int LeftTrigger = 1;
const unsigned int LeftBumper = 2;
const unsigned int RightTrigger = 3;
const unsigned int RightBumper = 4;
}  // namespace PoofsJoysticks

class ObservablePoofsJoystick;

class PoofsJoystickObserver {
public:
    PoofsJoystickObserver() {
    }
    virtual ~PoofsJoystickObserver() {
    }
    virtual void ObservePoofsJoystickStateChange(uint32_t port, uint32_t button,
                                                 bool newState) = 0;
};

class ObservablePoofsJoystick
        : public CoopTask
        , public Joystick {
public:
    static constexpr double DEADBAND_INPUT_THRESHOLD =
        0.05; /**< The deadband threshold on the joysticks. */
    static constexpr double VIRTUAL_JOYSTICK_THRESHOLD =
        0.5; /**< The virtual joystick threshold. */

protected:
    uint32_t m_port; /**< The port the joystick is plugged into. */

    /* For observer notification */
    PoofsJoystickObserver *m_observer; /**< The class to notify whenever a
                         change in the joystick occurs. */
    DriverStation *m_ds;               /**< The DriverStation operating on.*/
    uint32_t m_prevBtn;                /**< The previous button.*/
    TaskMgr *m_scheduler;              /**< The task manager object.*/
    LogCell *m_logCell;                /**< The logger.*/

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
     * Left X Virtual button.
     * @return Whether the left X virtual button is pressed.
     */
    bool GetLTrigger();

    /**
     * Left Y Virtual button.
     * @return Whether the left Y virtual button is pressed.
     */
    bool GetLBumper();

    /**
     * Right X Virtual button.
     * @return Whether the right X virtual button is pressed.
     */
    bool GetRTrigger();

    /**
     * Right Y Virtual button.
     * @return Whether the right Y virtual button is pressed.
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
};
}
