#include "src/AutonomousMode.h"

using namespace frc;

namespace frc973 {
// TODO: confirm intaking state sequencing with teleopmode
// TODO: Compare to fixed code. Add nessasary cases and then fix the case
// numbers.

void Autonomous::DoubleHatchAuto() {
    SingleHatchAuto();

    switch (m_autoStep) {
        case 7:
            m_drive->PIDTurn(m_tur * 135.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 8:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
        case 9:
            m_drive->PIDDrive(m_dir * 140.0, m_tur * 60.0,
                              Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 10:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }
            break;
            // case 12:
            //     if (m_drive->OnTarget()) {
            //         m_autoStep++;
            //     }
            //     break;
            // case 13:
            //     m_drive->PIDDrive(m_dir * 25.0, 0.0, Drive::RelativeTo::Now,
            //     0.8); m_autoStep++; break;
            // case 14:
            //     if (m_drive->OnTarget()) {
            //         m_autoStep++;
            //     }
            //     break;
            // case 15:
            //     m_drive->LimelightDriveWithSkew();
            //     m_autoStep++;
            //     break;
            // case 16:
            //     if (m_drive->OnTarget()) {
            //         m_autoStep++;
            //     }
            //     break;
            // case 17:
            //     m_hatchIntake->RunIntake();
            //     m_autoStep++;
            //     break;

            // case 18:
            //     m_drive->PIDDrive(m_dir * 10.0, 0.0, Drive::RelativeTo::Now,
            //     0.8); m_autoStep++; break;
            // case 19:
            //     if (m_drive->OnTarget()) {
            //         m_hatchIntake->SetIdle();
            //         m_autoStep++;
            //     }
            //     break;
            // case 20:
            //     m_drive->PIDTurn(m_tur * 180.0, Drive::RelativeTo::Now, 0.8);
            //     m_autoStep++;
            //     break;
            // case 21:
            //     if (m_drive->OnTarget()) {
            //         m_autoStep++;
            //     }
            //     break;
            // case 22:
            //     m_drive->PIDDrive(m_dir * 30.0, 0.0, Drive::RelativeTo::Now,
            //     0.8); m_autoStep++; break;
            // case 23:
            //     if (m_drive->OnTarget()) {
            //         m_autoStep++;
            //     }
            //     break;
            // case 24:
            //     m_drive->LimelightDriveWithSkew();
            //     m_autoStep++;
            //     break;
            // case 25:
            //     if (m_drive->OnTarget()) {
            //         m_autoStep++;
            //     }
            //     break;
            // case 26:
            //     m_hatchIntake->Exhaust();
            //     m_autoStep++;
            //     break;
            // case 27:
            //     m_drive->PIDDrive(m_dir * 10.0, 0.0, Drive::RelativeTo::Now,
            //     0.8); m_autoStep++; break;
            // case 28:
            //     if (m_drive->OnTarget()) {
            //         m_hatchIntake->SetIdle();
            //         m_autoStep++;
            //     }
            //     break;
    }
}
}
