#include "src/AutonomousMode.h"

using namespace frc;

namespace frc973 {

void Autonomous::SingleHatchAuto() {
    switch (m_autoStep) {
        case 0:
            m_drive->PIDDrive(m_dir * 50.0, 0.0, Drive::RelativeTo::Now, 0.5);
            m_autoStep++;
            break;
        case 1:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 2:
            if (m_autoStateStartPosition == AutoStateStartPosition::LeftHabLevel2) {
                m_drive->LimelightDriveWithSkew();
                m_limelightHatch->SetCameraVisionLeft();
                m_autoTimer = GetMsecTime();
                m_autoStep++;
                break;
            }
            else if (m_autoStateStartPosition == AutoStateStartPosition::RightHabLevel2) {
                m_drive->LimelightDriveWithSkew();
                m_limelightHatch->SetCameraVisionRight();
                m_autoTimer = GetMsecTime();
                m_autoStep++;
                break;
            }
        case 3:
            if (GetMsecTime() - m_autoTimer > 3000) {
                m_autoStep++;
            }
            break;
        case 4:
            m_hatchIntake->Exhaust();
            m_autoTimer = GetMsecTime();
            m_autoStep++;
            break;
        case 5:
            if (GetMsecTime() - m_autoTimer > 1000) {
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
