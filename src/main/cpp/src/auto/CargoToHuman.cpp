#include "src/AutonomousMode.h"

using namespace frc;

namespace frc973 {

void Autonomous::CargoToHuman() {
    switch (m_autoStep) {
        case 0:
            m_drive->PIDTurn(m_tur * 135.0, Drive::RelativeTo::Now, 0.8);
            m_autoStep++;
            break;
        case 1:
            if (m_drive->OnTarget()) {
                m_drive->PIDDrive(m_dir * 155.0, 0.0, Drive::RelativeTo::Now,
                                  0.8);
                m_autoStep++;
            }
            break;
        case 2:
            if (m_drive->OnTarget()) {
                m_drive->PIDTurn(m_tur * 45.0, Drive::RelativeTo::Now, 0.8);
                m_autoStep++;
            }
            break;
        case 3:
            if (m_drive->OnTarget()) {
                m_drive->LimelightDriveWithSkew();
                m_hatchIntake->RunIntake();
                m_autoStep++;
            }
            break;
        case 4:
            if (m_drive->OnTarget()) {
                m_hatchIntake->SetIdle();
                m_drive->PIDDrive(m_dir * -80.0, 0.0, Drive::RelativeTo::Now,
                                  0.8);
                m_autoStep++;
            }
            break;
        case 5:
            if (m_drive->OnTarget()) {
                m_drive->PIDTurn(m_tur * 180.0, Drive::RelativeTo::Now, 0.8);
                m_autoStep++;
            }
            break;
        case 6:
            if (m_drive->OnTarget()) {
            }
            break;
    }
}
}
