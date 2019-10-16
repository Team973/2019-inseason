#include "src/AutonomousMode.h"
#include "src/subsystems/HatchIntake.h"

namespace frc973 {

void Autonomous::CargoToHumanPlayer() {
    switch (m_autoStep) {
        case 0:  // Backwards -30"
            // m_drive->PIDDrive(-30.0, 0.0, DriveBase::RelativeTo::Absolute,
            // 0.8);
            m_autoStep++;
            break;
        case 1:  // turn toward human player right
                 // if (m_drive->OnTarget()) {
            // m_drive->PIDTurn(-90.0, DriveBase::RelativeTo::Absolute, 0.8);
            m_drive->PIDTurn(60.0, DriveBase::RelativeTo::Now, 0.7);
            m_autoStep++;
            // }
            break;
        case 2:  // ONLY IF NEEDED
            if (m_drive->OnTarget()) {
                // m_drive->PIDDrive(10.0, 0.0, DriveBase::RelativeTo::Now,
                // 0.8);
            }
            break;
        case 3:
            if (m_drive->OnTarget()) {
                m_drive->PIDTurn(-90.0, DriveBase::RelativeTo::Now, 0.8);
                m_autoStep++;
            }
            break;
        case 4:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 5:  // LimelightDriveWithSkew to human player
            m_drive->LimelightDriveWithSkew();

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 6:  // intake hatch
            m_hatchIntake->RunIntake();
            m_autoStep++;
            break;
        default:
            break;
    }
}
}
