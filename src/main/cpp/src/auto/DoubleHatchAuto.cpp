#include "src/AutonomousMode.h"

using namespace frc;

namespace frc973 {

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
            m_drive->PIDDrive(m_dir * 155.0, 0.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 10:
            if (m_drive->OnTarget()) {
                m_drive->PIDTurn(m_tur * 45.0, Drive::RelativeTo::Now, 0.8);
                m_autoStep++;
            }
            break;
        case 11:
            if (m_drive->OnTarget()) {
                m_drive->LimelightDriveWithSkew();
                m_autoStep++;
            }
            break;
        case 12:
            if (m_drive->OnTarget()) {
                m_hatchIntake->RunIntake();
                m_autoTimer = GetMsecTime();
                m_autoStep++;
            }
            break;
        case 13:
            if (GetMsecTime() - m_autoTimer > 250) {
                m_hatchIntake->SetIdle();
                m_drive->PIDDrive(m_dir * -80.0, 0.0, Drive::RelativeTo::Now, 0.8);
                m_autoStep++;
            }
            break;
        case 14:
            if (m_drive->OnTarget()) {
                m_drive->PIDTurn(m_tur * 180.0, Drive::RelativeTo::Now, 0.8);
                m_autoStep++;
            }
            break;
        case 15:
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
