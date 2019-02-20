/*
 * JoystickHelperBase.h
 *
 *  Created on: 2/19/19
 *      Author: John Pywtorak
 */
#pragma once

#include "frc/WPILib.h"

using namespace frc;

namespace frc973 {

namespace JoystickBase {

struct JoystickCommon {
    const unsigned int LeftXAxis;
    const unsigned int LeftYAxis;
    const unsigned int RightXAxis;
    const unsigned int RightYAxis;
    const unsigned int LeftBumper;
    const unsigned int RightBumper;
    const unsigned int LeftTrigger;
    const unsigned int RightTrigger;
};
}

class ObservableJoystickBase : public Joystick {
public:
    /**
     * Abstract Base class for the Observable...Joystick objects.
     */
    explicit ObservableJoystickBase(uint16_t port);
    virtual ~ObservableJoystickBase();

    /**
     * Get the value of the given axis with deadband.
     * @param axis Specifies the axis to get the value of.
     * @param fSquared Specifies whether the joystick input should be squared.
     * @param threshold Specifies the deadband threshold.
     */
    virtual float GetRawAxisWithDeadband(int axis, bool fSquared = false,
                                         double threshold = 0) = 0;

    /**
     * Get a const reference to the Common mapping for the type of joystick.
     */
    virtual const JoystickBase::JoystickCommon& GetJoystickCommon() = 0;
};
}
