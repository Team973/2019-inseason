#include "src/AutonomousMode.h"

using namespace frc;

namespace frc973 {
// TODO: confirm intaking state sequencing with teleopmode
// TODO: Compare to fixed code. Add nessasary cases and then fix the case
// numbers.

void Autonomous::DoubleHatchAuto() {
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
            m_autoTimer = GetMsecTime();
            m_autoStep++;
            break;
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
        case 7:
            m_drive->PIDTurn(-90.0, Drive::RelativeTo::Now, 0.8);

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 8:
            m_drive->PIDDrive(-30.0, 0.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 9:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 10:
            m_drive->PIDTurn(-90.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 11:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 12:
            m_drive->PIDDrive(25.0, 0.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 13:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 14:
            m_drive->LimelightDriveWithSkew();
            m_autoStep++;
            break;
        case 15:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 16:
            m_hatchIntake->RunIntake();
            m_autoStep++;
            break;

        case 17:
            m_drive->PIDDrive(-10.0, 0.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 18:
            if (m_drive->OnTarget()) {
                m_hatchIntake->SetIdle();
                m_autoStep++;
            }
            break;
        case 19:
            m_drive->PIDTurn(-180.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 20:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 21:
            m_drive->PIDDrive(30.0, 0.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 22:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 23:
            m_drive->LimelightDriveWithSkew();
            m_autoStep++;
            break;
        case 24:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 25:
            m_hatchIntake->Exhaust();
            m_autoStep++;
            break;

        case 26:
            m_drive->PIDDrive(-10.0, 0.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 27:
            if (m_drive->OnTarget()) {
                m_hatchIntake->SetIdle();
                m_autoStep++;
            }
            break;
    }
}
}
