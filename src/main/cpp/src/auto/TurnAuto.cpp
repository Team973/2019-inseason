#include "src/AutonomousMode.h"

using namespace frc;

namespace frc973 {

void Autonomous::TurnAuto() {
    switch (m_autoStep) {
        case 0:
            m_drive->PIDTurn(-90.0, Drive::RelativeTo::Now, 0.8);

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
    }
}
}
