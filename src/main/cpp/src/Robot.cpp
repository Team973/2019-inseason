#include "frc/WPILib.h"
#include <iostream>
#include "src/info/RobotInfo.h"
#include "src/DisabledMode.h"
#include "src/AutonomousMode.h"
#include "src/TeleopMode.h"
#include "src/TestMode.h"
#include "src/Robot.h"
#include "ctre/Phoenix.h"
#include "lib/util/WrapDash.h"
#include "src/controllers/LimelightDriveController.h"

using namespace frc;
using namespace ctre;
namespace frc973 {
Robot::Robot()
        : CoopMTRobot()
        , DualActionJoystickObserver()
        , PoofsJoystickObserver()
        , XboxJoystickObserver()
        , m_pdp(new PowerDistributionPanel())
        , m_driverJoystick(
              new ObservablePoofsJoystick(DRIVER_JOYSTICK_PORT, this, this))
        , m_operatorJoystick(
              new ObservableXboxJoystick(OPERATOR_JOYSTICK_PORT, this, this))
        , m_leftDriveTalonA(new GreyTalonSRX(LEFT_DRIVE_A_CAN_ID))
        , m_leftDriveVictorB(new VictorSPX(LEFT_DRIVE_B_VICTOR_ID))
        , m_rightDriveTalonA(new GreyTalonSRX(RIGHT_DRIVE_A_CAN_ID))
        , m_rightDriveVictorB(new VictorSPX(RIGHT_DRIVE_B_VICTOR_ID))
        , m_gyro(new ADXRS450_Gyro())
        , m_greylight(new GreyLight(NUM_LED))
        , m_limelightCargo(new Limelight("limelight-cargo"))
        , m_limelightHatch(new Limelight("limelight-hatch"))
        , m_logger(new LogSpreadsheet(this))
        , m_matchIdentifier(new LogCell("Match Identifier", 64))
        , m_gameSpecificMessage(new LogCell("GameSpecificMessage", 10))
        , m_drive(new Drive(this, m_logger, m_leftDriveTalonA,
                            m_leftDriveVictorB, m_rightDriveTalonA,
                            m_rightDriveVictorB, m_gyro, m_limelightCargo,
                            m_limelightHatch))
        , m_hatchIntake(new HatchIntake(this, m_logger))
        , m_airPressureSwitch(new DigitalInput(PRESSURE_DIN_ID))
        , m_compressorRelay(
              new Relay(COMPRESSOR_RELAY, Relay::Direction::kForwardOnly))
        , m_compressor(
              new GreyCompressor(m_airPressureSwitch, m_compressorRelay, this))
        , m_disabled(new Disabled(m_driverJoystick, m_operatorJoystick,
                                  m_greylight, m_limelightCargo,
                                  m_limelightHatch))
        , m_autonomous(new Autonomous(m_disabled, m_drive, m_gyro, m_greylight))
        , m_teleop(new Teleop(m_driverJoystick, m_operatorJoystick, m_drive,
                              m_hatchIntake, m_limelightCargo,
                              m_limelightHatch))
        , m_test(new Test(m_driverJoystick, m_operatorJoystick, m_drive,
                          m_hatchIntake, m_greylight)) {
    std::cout << "Constructed a Robot!" << std::endl;
}

Robot::~Robot() {
}

void Robot::Initialize() {
    m_compressor->Enable();
    m_logger->RegisterCell(m_matchIdentifier);
    m_logger->RegisterCell(m_gameSpecificMessage);
    m_logger->Start();
}

void Robot::DisabledStart() {
    m_disabled->DisabledInit();
}

void Robot::DisabledContinuous() {
    m_disabled->DisabledPeriodic();
}

void Robot::DisabledStop() {
    m_disabled->DisabledStop();
}

void Robot::AutonomousStart() {
    m_autonomous->AutonomousInit();
}

void Robot::AutonomousContinuous() {
    m_autonomous->AutonomousPeriodic();
}

void Robot::AutonomousStop() {
    m_autonomous->AutonomousStop();
}

void Robot::TeleopStart() {
    m_teleop->TeleopInit();
}

void Robot::TeleopContinuous() {
    m_teleop->TeleopPeriodic();
}

void Robot::TeleopStop() {
    m_teleop->TeleopStop();
}

void Robot::TestStart() {
    m_test->TestInit();
}

void Robot::TestContinuous() {
    m_test->TestPeriodic();
}

void Robot::TestStop() {
    m_test->TestStop();
}

void Robot::AllStateContinuous() {
    // NetworkTable Battery Voltage
    SmartDashboard::PutNumber("misc/pdp/batteryvoltage", m_pdp->GetVoltage());
    SmartDashboard::PutNumber("misc/limelight/cargo/target",
                              m_limelightCargo->isTargetValid());
    SmartDashboard::PutNumber("misc/limelight/hatch/target",
                              m_limelightHatch->isTargetValid());

    m_matchIdentifier->LogPrintf(
        "%s_%s%dm%d", DriverStation::GetInstance().GetEventName().c_str(),
        MatchTypeToString(DriverStation::GetInstance().GetMatchType()),
        DriverStation::GetInstance().GetMatchNumber(),
        DriverStation::GetInstance().GetReplayNumber());
    m_gameSpecificMessage->LogText(
        DriverStation::GetInstance().GetGameSpecificMessage().c_str());
    DBStringPrintf(DBStringPos::DB_LINE7, "Distance : %3.2lf",
                   m_limelightHatch->GetHorizontalDistance());
    DBStringPrintf(
        DBStringPos::DB_LINE8, "Pow(cos(offset)): %3.2lf",
        (pow(cos(m_limelightHatch->GetXOffset() * PI / 180 * 3.0), 5)));
}

void Robot::ObserveDualActionJoystickStateChange(uint32_t port, uint32_t button,
                                                 bool pressedP) {
    if (this->IsOperatorControl()) {
        m_teleop->HandleDualActionJoystick(port, button, pressedP);
    }
    else if (this->IsDisabled()) {
        m_disabled->HandleDualActionJoystick(port, button, pressedP);
    }
    else if (this->IsTest()) {
        m_test->HandleDualActionJoystick(port, button, pressedP);
    }
}

void Robot::ObservePoofsJoystickStateChange(uint32_t port, uint32_t button,
                                            bool pressedP) {
    if (this->IsOperatorControl()) {
        m_teleop->HandlePoofsJoystick(port, button, pressedP);
    }
    else if (this->IsTest()) {
        m_test->HandlePoofsJoystick(port, button, pressedP);
    }
}

void Robot::ObserveXboxJoystickStateChange(uint32_t port, uint32_t button,
                                           bool pressedP) {
    if (this->IsOperatorControl()) {
        m_teleop->HandleXboxJoystick(port, button, pressedP);
    }
    else if (this->IsDisabled()) {
        m_disabled->HandleXboxJoystick(port, button, pressedP);
    }
    else if (this->IsTest()) {
        m_test->HandleXboxJoystick(port, button, pressedP);
    }
}
}

int main() {
    return StartRobot<frc973::Robot>();
}
