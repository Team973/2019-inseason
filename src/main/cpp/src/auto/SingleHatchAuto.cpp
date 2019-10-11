#include "src/AutonomousMode.h"

using namespace frc;

namespace frc973 {

void Autonomous::SingleHatchAuto() {
    switch (m_autoStep) {
        case 0:
            m_drive->PIDDrive(25.0, 0.0, Drive::RelativeTo::Now, 0.8);

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;

        case 1:
            m_drive->LimelightDriveWithSkew();

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 2:
            m_hatchIntake->Exhaust();
            m_autoStep++;
            break;
        case 3:
            m_drive->PIDDrive(-10.0, 0.0, Drive::RelativeTo::Now, 0.8);
            if (m_drive->OnTarget()) {
                m_hatchIntake->SetIdle();
                m_autoStep++;
            }
            break;
    }
}
}
