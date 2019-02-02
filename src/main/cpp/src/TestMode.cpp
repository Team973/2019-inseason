#include "src/TestMode.h"

using namespace frc;

namespace frc973 {
Test::Test(ObservablePoofsJoystick *driver, ObservableXboxJoystick *codriver,
           Drive *drive, HatchIntake *hatchIntake, Elevator *elevator,
           CargoIntake *cargoIntake)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_drive(drive)
        , m_elevator(elevator)
        , m_hatchIntake(hatchIntake)
        , m_cargoIntake(cargoIntake)
        , m_cargoIntakeMotor(new GreyTalonSRX(5)) {
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
    double y = -m_driverJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis);
    double x =
        -m_driverJoystick->GetRawAxisWithDeadband(DualAction::RightXAxis);
    bool softwareLowGear =
        m_driverJoystick->GetRawButton(DualAction::RightTrigger);

    if (m_driveMode == DriveMode::Openloop) {
        if (softwareLowGear) {
            x /= 3.0;
            y /= 3.0;
        }
        m_drive->OpenloopArcadeDrive(y, x);
    }

    /**
     * Operator Joystick
     */

    double elevatorManualOutput =
        -m_operatorJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis);
    if (fabs(elevatorManualOutput) > 0.2) {  // manual
        m_elevator->SetManualInput();
    }
    else {  // motionmagic
    }

    DBStringPrintf(DBStringPos::DB_LINE4, "curr: %1.2lf out: %2.2lf",
                   m_cargoIntakeMotor->GetOutputCurrent(),
                   m_cargoIntakeMotor->GetBusVoltage());
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
            case PoofsJoysticks::RightBumper:
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
                    m_cargoIntakeMotor->Set(ControlMode::PercentOutput, -1.0);
                }
                else {
                    m_cargoIntakeMotor->Set(ControlMode::PercentOutput, -0.38);
                }
                break;
            case Xbox::BtnA:
                if (pressedP) {
                    m_cargoIntakeMotor->Set(ControlMode::PercentOutput, -0.38);
                }
                else {
                    m_cargoIntakeMotor->Set(ControlMode::PercentOutput, 0.0);
                }
                break;
            case Xbox::BtnX:
                if (pressedP) {
                }
                else {
                }
                break;
            case Xbox::BtnB:
                if (pressedP) {
                    m_cargoIntake->ExtendWrist();
                }
                else {
                    m_cargoIntake->RetractWrist();
                    m_elevator->SetPosition(Elevator::LOW_ROCKET_HATCH);
                }
                break;
            case Xbox::LeftBumper:
                if (pressedP) {
                    // m_cargoIntake->RunIntake();
                }
                else {
                    // m_cargoIntake->StopIntake();
                    // m_elevator->SetPosition(Elevator::MIDDLE_ROCKET_HATCH);
                }
                break;
            case Xbox::LJoystickBtn:
                if (pressedP) {
                    m_elevator->SetPosition(Elevator::HIGH_ROCKET_HATCH);
                }
                break;
            case Xbox::RJoystickBtn:
                if (pressedP) {
                    m_elevator->SetPosition(Elevator::LOW_ROCKET_CARGO);
                }
                break;
            case Xbox::RightBumper:
                if (pressedP) {
                    m_cargoIntake->Exhaust();
                }
                else {
                    m_cargoIntake->StopIntake();
                    m_elevator->SetPosition(Elevator::MIDDLE_ROCKET_CARGO);
                }
                break;
            case Xbox::DPadUpVirtBtn:
                if (pressedP) {
                    m_elevator->SetPosition(Elevator::HIGH_ROCKET_CARGO);
                }
                break;
            case Xbox::DPadDownVirtBtn:
                if (pressedP) {
                    m_cargoIntake->RunIntake(0.5);
                }
                else {
                    m_cargoIntake->RunIntake(0.0);
                    m_elevator->SetPosition(Elevator::CARGO_SHIP_HATCH);
                }
                break;
            case Xbox::DPadLeftVirtBtn:
                if (pressedP) {
                    m_elevator->SetPosition(Elevator::CARGO_SHIP_CARGO);
                }
                break;
            case Xbox::DPadRightVirtBtn:
                if (pressedP) {
                    m_elevator->SetPosition(Elevator::PLATFORM);
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
                m_hatchIntake->HatchOpen();
            }
            else {
                m_hatchIntake->HatchGrab();
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
                m_hatchIntake->HatchLaunch();
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
