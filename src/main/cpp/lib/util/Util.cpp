#include "lib/util/Util.h"
#include "frc/WPILib.h"
#include <stdio.h>

namespace frc973 {

RobotMode GetRobotMode(RobotState &stateProvider) {
    if (stateProvider.IsDisabled()) {
        return RobotMode::MODE_DISABLED;
    }
    else if (stateProvider.IsAutonomous()) {
        return RobotMode::MODE_AUTO;
    }
    else if (stateProvider.IsOperatorControl()) {
        return RobotMode::MODE_TELEOP;
    }
    else if (stateProvider.IsTest()) {
        return RobotMode::MODE_TEST;
    }
    else {
        printf(
            "Could not determine robot state... see lib::Util::GetRobotMode "
            ":(\n");
        return RobotMode::MODE_DISABLED;
    }
}
}
