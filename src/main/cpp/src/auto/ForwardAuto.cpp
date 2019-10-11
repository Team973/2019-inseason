#include "src/AutonomousMode.h"

using namespace frc;

namespace frc973 {

void Autonomous::ForwardAuto() {
    switch (m_autoStep) {
        case 0:
            m_drive->PIDDrive(24.0, 0.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 1:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
    }
}
}
