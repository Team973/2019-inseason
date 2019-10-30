#include "src/AutonomousMode.h"

using namespace frc;

namespace frc973 {

void Autonomous::DoubleHatchAuto() {
    StartToCargo();

    switch (m_autoStep) {
        case 5:
            m_drive->PIDTurn(m_tur * 135.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 6:
            if (m_drive->OnTarget()) {
                m_drive->PIDDrive(m_dir * 155.0, 0.0, Drive::RelativeTo::Now,
                                  0.8);
                m_autoStep++;
            }
            break;
        case 7:
            if (m_drive->OnTarget()) {
                m_drive->PIDTurn(m_tur * 45.0, Drive::RelativeTo::Now, 0.8);
                m_autoStep++;
            }
            break;
        case 8:
            if (m_drive->OnTarget()) {
                m_drive->LimelightDriveWithSkew();
                m_hatchIntake->RunIntake();
                m_autoStep++;
            }
            break;
        case 9:
            if (m_drive->OnTarget()) {
                m_hatchIntake->SetIdle();
                m_drive->PIDDrive(m_dir * -80.0, 0.0, Drive::RelativeTo::Now,
                                  0.8);
                m_autoStep++;
            }
            break;
        case 10:
            if (m_drive->OnTarget()) {
                m_drive->PIDTurn(m_tur * 180.0, Drive::RelativeTo::Now, 0.8);
                m_autoStep++;
            }
            break;
        case 11:
            if (m_drive->OnTarget()) {
                // m_autoStep++;
            }
            break;

            // case 22:
            //     if (m_drive->OnTarget()) {
            //         m_autoStep++;
            //     }
            //     break;
            // case 23:
            //     m_drive->PIDDrive(m_dir * 30.0, 0.0, Drive::RelativeTo::Now,
            //     0.8); m_autoStep++; break;
            // case 24:
            //     if (m_drive->OnTarget()) {
            //         m_autoStep++;
            //     }
            //     break;
            // case 25:
            //     m_drive->LimelightDriveWithSkew();
            //     m_autoStep++;
            //     break;
            // case 26:
            //     if (m_drive->OnTarget()) {
            //         m_autoStep++;
            //     }
            //     break;
            // case 27:
            //     m_hatchIntake->Exhaust();
            //     m_autoStep++;
            //     break;
            // case 28:
            //     m_drive->PIDDrive(m_dir * 10.0, 0.0, Drive::RelativeTo::Now,
            //     0.8); m_autoStep++;
            //     break;
            // case 29:
            //     if (m_drive->OnTarget()) {
            //         m_hatchIntake->SetIdle();
            //         m_autoStep++;
            //     }
            //     break;
    }
}
}
