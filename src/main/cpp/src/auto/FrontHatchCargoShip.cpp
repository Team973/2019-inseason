#include "src/AutonomousMode.h"
#include "src/auto/AutoCommon.h"

namespace frc973 {

/* enumerated AutoSteps
TurnClockwiseFromCargoScore,
TurnCounterClockwiseFromCargoScore,
DriveTowardHatchHumanLeft,
DriveTowardHatchHumanRight,
VisionFetchHumanHatch,
BackupFromRightHumanHatch,
BackupFromLeftHumanHatch,
DriveTowardFrontLowerRocket
*/

void Autonomous::CargoShipThenRocketAuto(const bool doCargoOnly) {
    double initial_dist = HAB_TO_FIELD_DISTANCE;
    double start_angle = m_drive->GetAngle();

    switch (m_autoStateStartPosition) {
        case AutoStateStartPosition::LeftHabLevel2:
            initial_dist += 12.0;
            break;
        case AutoStateStartPosition::CenterHab:
            initial_dist -= 12.0;
            break;
        case AutoStateStartPosition::RightHabLevel2:
            initial_dist += 12.0;
            break;
    }

    m_autoTimer = GetMsecTime();
    // m_driveMode in Autonomous? may not be needed

    switch (m_autoStep) {
        case 0:  // DriveForwardFromAny left, center, or rights (off the Hab)
            m_drive->PIDDrive(initial_dist, 0.0,
                              DriveBase::RelativeTo::Absolute, 1.0);
            m_autoStep++;
            break;
        case 1:  // TurnTowardCargoFront
            if (m_drive->OnTarget()) {
                m_drive->PIDTurn(HAB_TURN_TOWARD_TARGET,
                                 DriveBase::RelativeTo::Now, 1.0);
                m_autoStep++;
            }

            break;
        case 2:  // VisionCargoFrontInit
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }

            break;
        case 3:  // VisionCargoFront
            m_drive->LimelightDriveWithSkew();

            if (m_drive->OnTarget()) {
                m_autoStep++;
            }

            break;
        case 4:  // ScoreCargoFront
            m_hatchIntake->Exhaust();
            m_autoStep++;
            break;
        case 5:  // BackupCargoFront
            m_drive->PIDDrive(-12.0, 0.0, DriveBase::RelativeTo::Now, 1.0);
            m_autoStep++;
            break;
        case 6:
            if (m_drive->OnTarget()) {
                m_autoStep++;
            }

            break;
        default:
            break;
    }
}

void Autonomous::TwoCargoShipAuto() {
    if (m_autoStep < 7) {
        CargoShipThenRocketAuto(true);
    }
}
}
