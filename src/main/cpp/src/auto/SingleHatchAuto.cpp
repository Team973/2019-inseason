#include "src/AutonomousMode.h"

using namespace frc;

namespace frc973 {

void Autonomous::SingleHatchAuto() {
    switch (m_autoStep) {
        case 0:
            m_drive->PIDDrive(-50.0, 0.0, Drive::RelativeTo::Now, 0.5);
            m_autoStep++;
            break;
        case 1:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 2:
            m_drive->LimelightDriveWithSkew();
            m_limelightHatch->SetCameraVisionRight();
            timer = GetMsecTime();
            m_autoStep++;
            break;
        case 3:
            if (GetMsecTime() - timer > 3000) {
                m_autoStep++;
            }
            break;
        case 4:
            m_hatchIntake->Exhaust();
            timer = GetMsecTime();
            m_autoStep++;
            break;
        case 5:
            if (GetMsecTime() - timer > 1000) {
                m_drive->PIDDrive(20.0, 0.0, Drive::RelativeTo::Now, 0.8);
                m_autoStep++;
            }
            break;
        case 6:
            if (m_drive->OnTarget()) {
                m_hatchIntake->SetIdle();
                m_autoStep++;
            }
            break;
    }
}
}
