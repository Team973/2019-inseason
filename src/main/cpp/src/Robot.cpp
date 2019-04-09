#include "frc/WPILib.h"
#include <iostream>
#include <time.h>
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
        , m_testJoystick(
              new ObservableDualActionJoystick(TEST_JOYSTICK_PORT, this, this))
        , m_logger(new LogSpreadsheet(this))
        , m_leftDriveSparkA(new GreySparkMax(
              LEFT_DRIVE_A_ID, CANSparkMax::MotorType::kBrushless))
        , m_leftDriveSparkB(new GreySparkMax(
              LEFT_DRIVE_B_ID, CANSparkMax::MotorType::kBrushless))
        , m_leftDriveSparkC(new GreySparkMax(
              LEFT_DRIVE_C_ID, CANSparkMax::MotorType::kBrushless))
        , m_rightDriveSparkA(new GreySparkMax(
              RIGHT_DRIVE_A_ID, CANSparkMax::MotorType::kBrushless))
        , m_rightDriveSparkB(new GreySparkMax(
              RIGHT_DRIVE_B_ID, CANSparkMax::MotorType::kBrushless))
        , m_rightDriveSparkC(new GreySparkMax(
              RIGHT_DRIVE_C_ID, CANSparkMax::MotorType::kBrushless))
        , m_stingerDriveMotor(new GreyTalonSRX(STINGER_DRIVE_CAN_ID))
        , m_elevatorMotorA(new GreyTalonSRX(ELEVATOR_A_CAN_ID))
        , m_elevatorMotorB(new VictorSPX(ELEVATOR_B_CAN_ID))
        , m_elevatorHall(new DigitalInput(ELEVATOR_HALL_ID))
        , m_gyro(new ADXRS450_Gyro())
        , m_hatchCamera(UsbCamera("USB Cmera 0", 0))
        , m_cameraServer(CameraServer::GetInstance())
        , m_greyCam(m_cameraServer->AddServer("serve_GreyCam", 1181))
        , m_cargoIntakeMotor(new GreyTalonSRX(CARGO_INTAKE_CAN_ID))
        , m_cargoWrist(new Solenoid(PCM_CAN_ID, CARGO_INTAKE_WRIST_PCM_ID))
        , m_cargoPlatformLock(
              new Solenoid(PCM_CAN_ID, CARGO_PLATFORM_LOCK_PCM_ID))
        , m_hatchRollers(new GreyTalonSRX(HATCH_ROLLER_CAN_ID))
        , m_pigeonGyro(new PigeonIMU(m_hatchRollers))
        , m_hatchPuncher(new Solenoid(PCM_CAN_ID, HATCH_PUNCHER_PCM_ID))
        , m_limelightHatch(new Limelight("limelight-hatch", false))
        , m_matchIdentifier(new LogCell("Match Identifier", 64))
        , m_batteryVoltage(new LogCell("Battery Voltage", 32, true))
        , m_matchTime(new LogCell("MatchTime", 32, true))
        , m_dateTime(new LogCell("Date and Time", 32, true))
        , m_hatchIntake(new HatchIntake(this, m_logger, m_hatchRollers,
                                        m_hatchPuncher, m_limelightHatch))
        , m_elevator(new Elevator(this, m_logger, m_elevatorMotorA,
                                  m_elevatorMotorB, m_operatorJoystick,
                                  m_elevatorHall))
        , m_drive(new Drive(
              this, m_logger, m_leftDriveSparkA, m_leftDriveSparkB,
              m_leftDriveSparkC, m_rightDriveSparkA, m_rightDriveSparkB,
              m_rightDriveSparkC, m_stingerDriveMotor, m_gyro, m_pigeonGyro, m_limelightHatch,
              m_hatchIntake, m_elevator, m_driverJoystick, m_operatorJoystick))
        , m_cargoIntake(new CargoIntake(this, m_logger, m_cargoIntakeMotor,
                                        m_cargoPlatformLock, m_cargoWrist,
                                        m_limelightHatch))
        , m_stinger(new Stinger(this, m_logger, m_stingerDriveMotor))
        , m_airPressureSwitch(new DigitalInput(PRESSURE_DIN_ID))
        , m_compressorRelay(
              new Relay(COMPRESSOR_RELAY, Relay::Direction::kForwardOnly))
        , m_compressor(
              new GreyCompressor(m_airPressureSwitch, m_compressorRelay, this))
        , m_disabled(new Disabled(m_driverJoystick, m_operatorJoystick,
                                  m_elevator, m_cargoIntake, m_limelightHatch,
                                  m_autonomous))
        , m_teleop(new Teleop(m_driverJoystick, m_operatorJoystick,
                              m_testJoystick, m_drive, m_elevator,
                              m_hatchIntake, m_cargoIntake, m_stinger,
                              m_limelightHatch))
        , m_test(new Test(m_driverJoystick, m_operatorJoystick, m_testJoystick,
                          m_drive, m_elevator, m_hatchIntake, m_cargoIntake,
                          m_stinger, m_limelightHatch))
        , m_autonomous(new Autonomous(
              m_driverJoystick, m_operatorJoystick, m_testJoystick, m_teleop,
              m_gyro, m_drive, m_cargoIntake, m_hatchIntake, m_elevator)) {
    std::cout << "Constructed a Robot!" << std::endl;
}

Robot::~Robot() {
}

void Robot::Initialize() {
    m_compressor->Enable();
    m_logger->RegisterCell(m_matchIdentifier);
    m_logger->RegisterCell(m_batteryVoltage);
    m_logger->RegisterCell(m_matchTime);
    m_logger->RegisterCell(m_dateTime);
    m_logger->Start();

    m_cameraServer->AddCamera(m_hatchCamera);
    m_hatchCamera.SetVideoMode(VideoMode::PixelFormat::kMJPEG, 160, 120, 10);
    m_greyCam.SetSource(m_hatchCamera);

    m_pigeonGyro->SetFusedHeading(0,10);
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
    time_t t;
    struct tm *tmp;
    char TIME_BUFFER[50];
    time(&t);
    tmp = localtime(&t);
    strftime(TIME_BUFFER, sizeof(TIME_BUFFER), "%g%m%d %H:%M", tmp);

    m_matchIdentifier->LogPrintf(
        "%s_%s%dm%d", DriverStation::GetInstance().GetEventName().c_str(),
        MatchTypeToString(DriverStation::GetInstance().GetMatchType()),
        DriverStation::GetInstance().GetMatchNumber(),
        DriverStation::GetInstance().GetReplayNumber());
    m_batteryVoltage->LogDouble(m_pdp->GetVoltage());
    m_matchTime->LogDouble(GetMsecTime());
    m_dateTime->LogPrintf("%s", TIME_BUFFER);

    /*m_limelightHatch->SetLightOn();
    m_limelightHatch->SetCameraVisionCenter();
    // m_limelightHatch->SetCameraVisionLeft();
    // m_limelightHatch->SetCameraVisionRight();
    DBStringPrintf(DB_LINE7, "td:%2.2lf xo:%2.2lf",
                   m_limelightHatch->GetHorizontalDistance(),
                   m_limelightHatch->GetXOffset());*/

    SmartDashboard::PutNumber("misc/pdp/voltage", m_pdp->GetVoltage());
    SmartDashboard::PutNumber(
        "limelight/throttle",
        m_drive->GetLimelightDriveWithSkew()->GetThrottlePidOut());
    SmartDashboard::PutNumber(
        "limelight/turn",
        m_drive->GetLimelightDriveWithSkew()->GetTurnPidOut());
    SmartDashboard::PutNumber(
        "limelight/skewcont",
        m_drive->GetLimelightDriveWithSkew()->GetGoalAngleComp());

    SmartDashboard::PutNumber("limelight/xoff", m_limelightHatch->GetXOffset());
    SmartDashboard::PutNumber("limelight/distance",
                              m_limelightHatch->GetHorizontalDistance());
    SmartDashboard::PutNumber("limelight/skew",
                              m_limelightHatch->GetTargetSkew());
    SmartDashboard::PutBoolean("limelight/target",
                               m_limelightHatch->isTargetValid());
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
    else if (this->IsAutonomous()) {
        m_autonomous->HandleDualActionJoystick(port, button, pressedP);
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
    else if (this->IsAutonomous()) {
        m_autonomous->HandlePoofsJoystick(port, button, pressedP);
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
    else if (this->IsAutonomous()) {
        m_autonomous->HandleXboxJoystick(port, button, pressedP);
    }
}
}
int main() {
    return StartRobot<frc973::Robot>();
}
