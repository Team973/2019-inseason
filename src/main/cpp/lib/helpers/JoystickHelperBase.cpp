#include "lib/helpers/JoystickHelperBase.h"

namespace frc973 {

ObservableJoystickBase::ObservableJoystickBase(uint16_t port)
        : Joystick(port), m_joystick() {
}

ObservableJoystickBase::~ObservableJoystickBase() {
}

const JoystickBase::Joystick &ObservableJoystickBase::GetPeriodicJoystick()
    const {
    return this->m_joystick;
}

void ObservableJoystickBase::SetPeriodicJoystick(
    JoystickBase::Joystick stickStruct) {
    m_joystick.LeftYAxis = stickStruct.LeftYAxis;
    m_joystick.RightXAxis = stickStruct.RightXAxis;
    m_joystick.RightBumper = stickStruct.RightBumper;
    m_joystick.RightTrigger = stickStruct.RightTrigger;
}
}
