#include "src/TestMode.h"

using namespace frc;

namespace frc973 {
Test::Test(ObservablePoofsJoystick *driver, ObservableXboxJoystick *codriver,
           Drive *drive, Elevator *elevator, HatchIntake *hatchIntake,
           CargoIntake *cargoIntake, Stinger *stinger,
           Limelight *limelightCargo, Limelight *limelightHatch)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_drive(drive)
        , m_elevator(elevator)
        , m_driveMode(DriveMode::Cheesy)
        , m_hatchIntake(hatchIntake)
        , m_cargoIntake(cargoIntake)
        , m_stinger(stinger)
        , m_limelightCargo(limelightCargo)
        , m_limelightHatch(limelightHatch)
        , m_rumble(Rumble::off) {
}

Test::~Test() {
}

void Test::TestInit() {
    std::cout << "Test Start" << std::endl;
    m_elevator->EnableCoastMode();
    m_driveMode = DriveMode::Openloop;
}

void Test::TestPeriodic() {
    /**
     * Driver Joystick
     */
    double y = m_operatorJoystick->GetRawAxisWithDeadband(Xbox::LeftYAxis);
    //-m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::LeftYAxis);
    double x = m_operatorJoystick->GetRawAxisWithDeadband(Xbox::RightXAxis);
    //-m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::RightXAxis);
    bool quickturn =
        m_driverJoystick->GetRawButton(PoofsJoysticks::RightBumper);

    bool softwareLowGear =
        m_driverJoystick->GetRawButton(PoofsJoysticks::RightTrigger);

    switch (m_driveMode) {
        case DriveMode::Cheesy:
            if (softwareLowGear) {
                m_drive->CheesyDrive(y / 3.0, x / 3.0, quickturn, false);
            }
            else {
                m_drive->CheesyDrive(y, x, quickturn, false);
            }
            break;
        case DriveMode::Openloop:
            if (softwareLowGear) {
                m_drive->OpenloopArcadeDrive(y / 3.0, x / 3.0);
            }
            else {
                m_drive->OpenloopArcadeDrive(y, x);
            }
            break;
        case DriveMode::LimelightCargo:
            m_drive->LimelightCargoDrive();
            break;
        case DriveMode::LimelightHatch:
            m_drive->LimelightHatchDrive();
            break;
        case DriveMode::AssistedCheesy:
            m_drive->AssistedCheesyDrive(y, x, quickturn, false);
            break;
    }

    /**
     * Operator Joystick
     */
    switch (m_rumble) {
        case Rumble::on:
            m_rumbleTimer = GetMsecTime();
            m_operatorJoystick->SetRumble(GenericHID ::RumbleType::kRightRumble,
                                          1);
            m_operatorJoystick->SetRumble(GenericHID ::RumbleType::kLeftRumble,
                                          0);
            break;
        case Rumble::off:
            if ((GetMsecTime() - m_rumbleTimer) > 150) {
                m_operatorJoystick->SetRumble(
                    GenericHID ::RumbleType::kRightRumble, 0);
                m_operatorJoystick->SetRumble(
                    GenericHID ::RumbleType::kLeftRumble, 0);
            }
            break;
    }

    double elevatorManualOutput =
        -m_operatorJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis);
    if (fabs(elevatorManualOutput) > 0.2) {  // manual
        m_elevator->SetManualInput();
    }
    else {  // motionmagic
    }
}

void Test::TestStop() {
}

void Test::HandlePoofsJoystick(uint32_t port, uint32_t button, bool pressedP) {
    if (port == DRIVER_JOYSTICK_PORT) {
        switch (button) {
            case PoofsJoysticks::LeftTrigger:
                if (pressedP) {
                }
                else {
                }
                break;
            case PoofsJoysticks::RightTrigger:
                if (pressedP) {
                }
                else {
                }
                break;
            case PoofsJoysticks::LeftBumper:
                if (pressedP) {
                }
                else {
                }
                break;
            case PoofsJoysticks::RightBumper:  // Quickturn
                if (pressedP) {
                }
                else {
                }
                break;
        }
    }
}

void Test::HandleXboxJoystick(uint32_t port, uint32_t button, bool pressedP) {
    if (port == OPERATOR_JOYSTICK_PORT) {
        switch (button) {
            case Xbox::BtnY:
                if (pressedP) {
                    m_hatchIntake->RunIntake();
                }
                else {
                    m_hatchIntake->HoldHatch();
                }
                break;
            case Xbox::BtnA:
                if (pressedP) {
                    m_hatchIntake->Exhaust();
                }
                else {
                    m_hatchIntake->SetIdle();
                }
                break;
            case Xbox::BtnX:
                if (pressedP) {
                    m_cargoIntake->DeployPlatformWheel();
                }
                else {
                    m_cargoIntake->RetractPlatformWheel();
                }
            case Xbox::BtnB:
                if (pressedP) {
                    m_hatchIntake->ManualPuncherActivate();
                }
                else {
                    m_hatchIntake->ManualPuncherRetract();
                }
                break;
            case Xbox::LeftBumper:
                if (pressedP) {
                    m_cargoIntake->RunIntake();
                }
                else {
                }
                break;
            case Xbox::LJoystickBtn:
                if (pressedP) {
                }
                break;
            case Xbox::RJoystickBtn:
                if (pressedP) {
                }
                break;
            case Xbox::RightBumper:
                if (pressedP) {
                    m_cargoIntake->Exhaust();
                }
                else {
                    m_cargoIntake->StopIntake();
                }
                break;
            case Xbox::DPadUpVirtBtn:  // Changes game mode to Endgame
                if (pressedP) {
                    m_stinger->SetPower(-1.0);
                    m_elevator->SetPower(-0.5);
                }
                else {
                    m_stinger->SetPower(0.0);
                    m_elevator->SetPower(0.0);
                }
                break;
            case Xbox::DPadDownVirtBtn:
                if (pressedP) {
                    m_cargoIntake->RunIntake(-1.0);
                }
                else {
                    m_cargoIntake->RunIntake(0.0);
                }
                break;
            case Xbox::DPadLeftVirtBtn:
                if (pressedP) {
                    m_rumble = Rumble::on;
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case Xbox::DPadRightVirtBtn:
                if (pressedP) {
                    m_rumble = Rumble::on;
                }
                else {
                    m_rumble = Rumble::off;
                }
                break;
            case Xbox::Back:
                if (pressedP) {
                }
                else {
                }
                break;
            case Xbox::Start:
                if (pressedP) {
                }
                else {
                }
                break;
        }
    }
}

void Test::HandleDualActionJoystick(uint32_t port, uint32_t button,
                                    bool pressedP) {
    switch (button) {
        case DualAction::BtnA:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::BtnY:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::LeftBumper:
            if (pressedP) {
                m_cargoIntake->RunIntake();
            }
            else {
                m_cargoIntake->StopIntake();
            }
            break;
        case DualAction::LeftTrigger:
            if (pressedP) {
                m_cargoIntake->Exhaust();
            }
            else {
                m_cargoIntake->StopIntake();
            }
            break;
        case DualAction::RightBumper:
            if (pressedP) {
                m_cargoIntake->ExtendWrist();
            }
            else {
                m_cargoIntake->RetractWrist();
            }
            break;
        case DualAction::RightTrigger:
            if (pressedP) {
                m_cargoIntake->DeployPlatformWheel();
            }
            else {
                m_cargoIntake->RetractPlatformWheel();
            }
            break;
        case DualAction::DPadUpVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::DPadDownVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::DPadLeftVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        case DualAction::DPadRightVirtBtn:
            if (pressedP) {
            }
            else {
            }
            break;
        default:
            break;
    }
}
}
