#include "lib/bases/JoystickHelperBase.h"

namespace frc973 {

ObservableJoystickBase::ObservableJoystickBase(uint16_t port) : Joystick(port) {
}

ObservableJoystickBase::~ObservableJoystickBase() {
}
}
