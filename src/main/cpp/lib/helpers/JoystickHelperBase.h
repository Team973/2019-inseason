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

typedef struct {
    unsigned int LeftYAxis;
    unsigned int RightXAxis;
    unsigned int RightBumper;
    unsigned int RightTrigger;
    // ...
} Joystick;
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

    const JoystickBase::Joystick &GetPeriodicJoystick() const;

protected:
    void SetPeriodicJoystick(JoystickBase::Joystick stickStruct);

    JoystickBase::Joystick m_joystick;
};
}
