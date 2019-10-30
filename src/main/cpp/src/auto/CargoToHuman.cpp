#include "src/AutonomousMode.h"
#include "src/subsystems/HatchIntake.h"

namespace frc973 {
void Autonomous::StartPlatformToCargo() {
    switch (m_autoStep) {
        case 0:  // foward 50"
            m_drive->PIDDrive(-50.0, 0.0, DriveBase::RelativeTo::Now, 0.5);
            m_autoStep++;
            break;
        case 1:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
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
                m_autoStep++;
            }
            break;
        default:
            break;
    }
}

void Autonomous::CargoToHumanPlayer() {
    switch (m_autoStep) {
        case 6:  // Backwards 30"
            m_drive->PIDDrive(30.0, 0.0, DriveBase::RelativeTo::Absolute, 0.8);
            m_autoStep++;
            break;
        case 7:  // turn toward human player right
            if (m_drive->OnTarget()) {
                m_hatchIntake->SetIdle();
                m_drive->PIDTurn(-90.0, DriveBase::RelativeTo::Absolute, 0.8);
                // m_drive->PIDTurn(60.0, DriveBase::RelativeTo::Now, 0.7);
                m_autoStep++;
            }
            break;
        case 8:  // ONLY IF NEEDED
            if (m_drive->OnTarget()) {
                m_drive->PIDDrive(10.0, 0.0, DriveBase::RelativeTo::Now, 0.8);
            }
            break;
        case 9:
            if (m_drive->OnTarget()) {
                m_drive->PIDTurn(-90.0, DriveBase::RelativeTo::Now, 0.8);
                m_autoStep++;
            }
            break;
        case 10:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 11:  // LimelightDriveWithSkew to human player
            m_drive->LimelightDriveWithSkew();

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 12:  // intake hatch
            m_hatchIntake->RunIntake();
            m_autoStep++;
            break;
        default:
            break;
    }
}
}
