#include "src/AutonomousMode.h"

using namespace frc;

namespace frc973 {
// TODO: confirm intaking state sequencing with teleopmode
void Autonomous::DoubleHatchAuto() {
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

        case 4:
            m_drive->PIDTurn(-90.0, Drive::RelativeTo::Now, 0.8);

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;

        case 5:
            m_drive->PIDDrive(30.0, 0.0, Drive::RelativeTo::Now, 0.8);

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;

        case 6:
            m_drive->PIDTurn(-90.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;

        case 7:
            m_drive->PIDDrive(25.0, 0.0, Drive::RelativeTo::Now, 0.8);

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;

        case 8:
            m_drive->LimelightDriveWithSkew();

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;

        case 9:
            m_hatchIntake->RunIntake();
            m_autoStep++;
            break;

        case 10:
            m_drive->PIDDrive(-10.0, 0.0, Drive::RelativeTo::Now, 0.8);

            if (m_drive->OnTarget()) {
                m_hatchIntake->SetIdle();
                m_autoStep++;
            }
            break;

        case 11:
            m_drive->PIDTurn(-180.0, Drive::RelativeTo::Now, 0.8);
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;

        case 12:
            m_drive->PIDDrive(30.0, 0.0, Drive::RelativeTo::Now, 0.8);

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;

        case 13:
            m_drive->LimelightDriveWithSkew();

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;

        case 14:
            m_hatchIntake->Exhaust();
            m_autoStep++;
            break;

        case 15:
            m_drive->PIDDrive(-10.0, 0.0, Drive::RelativeTo::Now, 0.8);

            if (m_drive->OnTarget()) {
                m_hatchIntake->SetIdle();
                m_autoStep++;
            }
            break;
    }
}
}
