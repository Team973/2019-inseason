/*
 * Drive.cpp
 *
 *  Created on: January 7, 2019
 *      Authors: Luis and Dylan
 */

#include "src/subsystems/Drive.h"

namespace frc973 {

using namespace Trajectories;

Drive::Drive(TaskMgr *scheduler, LogSpreadsheet *logger,
             GreySparkMax *leftDriveSparkA, GreySparkMax *leftDriveSparkB,
             GreySparkMax *leftDriveSparkC, GreySparkMax *rightDriveSparkA,
             GreySparkMax *rightDriveSparkB, GreySparkMax *rightDriveSparkC,
             GreyTalonSRX *stingerDriveMotor, ADXRS450_Gyro *gyro,
             Limelight *limelightHatch, HatchIntake *hatchIntake,
             Elevator *elevator, ObservablePoofsJoystick *driverJoystick,
             ObservableXboxJoystick *operatorJoystick)
        : DriveBase(scheduler, this, this, nullptr)
        , m_logger(logger)
        , m_leftDriveSparkA(leftDriveSparkA)
        , m_leftDriveSparkB(leftDriveSparkB)
        , m_leftDriveSparkC(leftDriveSparkC)
        , m_rightDriveSparkA(rightDriveSparkA)
        , m_rightDriveSparkB(rightDriveSparkB)
        , m_rightDriveSparkC(rightDriveSparkC)
        , m_stingerDriveMotor(stingerDriveMotor)
        , m_controlMode(ControlMode::PercentOutput)
        , m_leftDriveOutput(0.0)
        , m_rightDriveOutput(0.0)
        , m_leftDriveOutputLog(new LogCell("Left motor signal (pow or vel)"))
        , m_rightDriveOutputLog(new LogCell("Right motor signal (pow or vel)"))
        , m_stingerDriveOutputLog(
              new LogCell("Stinger motor signal (pow or vel)"))
        , m_driveControllerLog(new LogCell("Drive Controller Active"))
        , m_leftVoltageLog(new LogCell("Left motor voltage"))
        , m_rightVoltageLog(new LogCell("Right motor voltage"))
        , m_stingerVoltageLog(new LogCell("Stinger motor voltage"))
        , m_targetLog(new LogCell("LL Target Valid?"))
        , m_xOffsetLog(new LogCell("LL X Offset"))
        , m_yOffsetLog(new LogCell("LL Y Offset"))
        , m_targetAreaLog(new LogCell("LL Target Area"))
        , m_targetSkewLog(new LogCell("LL Target Skew"))
        , m_latencyLog(new LogCell("LL Latency"))
        , m_pipelineLog(new LogCell("LL Pipeline"))
        , m_horizontalLengthLog(new LogCell("LL Horizontal Length"))
        , m_verticalLengthLog(new LogCell("LL Vertical Length"))
        , m_horizontalDistanceLog(new LogCell("LL Horizontal Distance"))
        , m_leftPosZero(0.0)
        , m_rightPosZero(0.0)
        , m_gyro(gyro)
        , m_limelightHatch(limelightHatch)
        , m_hatchIntake(hatchIntake)
        , m_elevator(elevator)
        , m_driverJoystick(driverJoystick)
        , m_operatorJoystick(operatorJoystick)
        , m_cheesyDriveController(new CheesyDriveController(limelightHatch))
        , m_openloopArcadeDriveController(new OpenloopArcadeDriveController())
        , m_pidDriveController(new PIDDriveController())
        , m_splineDriveController(new SplineDriveController(this, logger))
        , m_constantArcSplineDriveController(
              new ConstantArcSplineDriveController(this, logger))
        , m_velocityArcadeDriveController(new VelocityArcadeDriveController())
        , m_limelightDriveWithoutSkew(new LimelightDriveController(
              logger, limelightHatch, false, m_driverJoystick, m_hatchIntake,
              m_elevator))
        , m_limelightDriveWithSkew(new LimelightDriveController(
              logger, limelightHatch, true, m_driverJoystick, m_hatchIntake,
              m_elevator))
        , m_assistedCheesyDriveHatchController(
              new AssistedCheesyDriveController(
                  m_limelightHatch,
                  AssistedCheesyDriveController::VisionOffset::Hatch))
        , m_angle()
        , m_angleRate()
        , m_angleLog(new LogCell("Angle"))
        , m_angularRateLog(new LogCell("Angular Rate"))
        , m_leftDistLog(new LogCell("Left Encoder Distance"))
        , m_leftDistRateLog(new LogCell("Left Encoder Rate"))
        , m_rightDistLog(new LogCell("Right Encoder Distance"))
        , m_rightDistRateLog(new LogCell("Right Encoder Rate"))
        , m_currentLog(new LogCell("Drive current")) {
    this->m_scheduler->RegisterTask("Drive", this, TASK_PERIODIC);
    m_leftDriveSparkA->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_leftDriveSparkA->SetInverted(false);
    m_leftDriveSparkA->SetOpenLoopRampRate(0.3);
    m_leftDriveSparkA->EnableVoltageCompensation(12.0);
    m_leftDriveSparkA->Config_PID(0, 1.0, 0.0, 0.0, 0.0);

    m_leftDriveSparkB->Follow(*m_leftDriveSparkA);
    m_leftDriveSparkB->SetInverted(false);
    m_leftDriveSparkC->Follow(*m_leftDriveSparkB);
    m_leftDriveSparkC->SetInverted(false);

    m_rightDriveSparkA->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_rightDriveSparkA->SetInverted(false);
    m_rightDriveSparkA->SetOpenLoopRampRate(0.3);
    m_rightDriveSparkA->EnableVoltageCompensation(12.0);
    m_rightDriveSparkA->Config_PID(0, 0.01, 0.0, 0.0, 0.0);

    m_rightDriveSparkB->Follow(*m_rightDriveSparkA);
    m_rightDriveSparkB->SetInverted(false);
    m_rightDriveSparkC->Follow(*m_rightDriveSparkB);
    m_rightDriveSparkC->SetInverted(false);

    m_stingerDriveMotor->SetNeutralMode(Coast);
    m_stingerDriveMotor->SetInverted(false);

    m_stingerDriveMotor->EnableCurrentLimit(true);
    m_stingerDriveMotor->ConfigPeakCurrentDuration(0, 10);
    m_stingerDriveMotor->ConfigPeakCurrentLimit(0, 10);
    m_stingerDriveMotor->ConfigContinuousCurrentLimit(40, 10);

    logger->RegisterCell(m_angleLog);
    logger->RegisterCell(m_angularRateLog);
    logger->RegisterCell(m_leftDistLog);
    logger->RegisterCell(m_leftDistRateLog);
    logger->RegisterCell(m_rightDistLog);
    logger->RegisterCell(m_rightDistRateLog);
    logger->RegisterCell(m_currentLog);
    logger->RegisterCell(m_driveControllerLog);

    logger->RegisterCell(m_targetLog);
    logger->RegisterCell(m_xOffsetLog);
    logger->RegisterCell(m_yOffsetLog);
    logger->RegisterCell(m_targetAreaLog);
    logger->RegisterCell(m_targetSkewLog);
    logger->RegisterCell(m_latencyLog);
    logger->RegisterCell(m_pipelineLog);
    logger->RegisterCell(m_horizontalLengthLog);
    logger->RegisterCell(m_verticalLengthLog);
    logger->RegisterCell(m_horizontalDistanceLog);
}

Drive::~Drive() {
}

LimelightDriveController *Drive::GetLimelightDriveWithSkew() const {
    return m_limelightDriveWithSkew;
}

void Drive::CheesyDrive(double throttle, double turn, bool isQuickTurn,
                        bool isHighGear) {
    this->SetDriveController(m_cheesyDriveController);
    m_cheesyDriveController->SetJoysticks(throttle, turn, isQuickTurn,
                                          isHighGear);
    m_driveControllerLog->LogPrintf("Cheesy Drive");
}

OpenloopArcadeDriveController *Drive::OpenloopArcadeDrive(double throttle,
                                                          double turn) {
    this->SetDriveController(m_openloopArcadeDriveController);
    m_openloopArcadeDriveController->SetJoysticks(throttle, turn);
    m_driveControllerLog->LogPrintf("OpenLoopArcade Drive");

    return m_openloopArcadeDriveController;
}

PIDDriveController *Drive::PIDDrive(double dist, double turn,
                                    RelativeTo relativity, double powerCap) {
    this->SetDriveController(m_pidDriveController);
    m_pidDriveController->SetCap(powerCap);
    m_pidDriveController->SetTarget(dist, turn, relativity, this);
    m_driveControllerLog->LogPrintf("PID Drive");

    return m_pidDriveController;
}

PIDDriveController *Drive::PIDTurn(double turn, RelativeTo relativity,
                                   double powerCap) {
    this->SetDriveController(m_pidDriveController);
    m_pidDriveController->SetCap(powerCap);
    m_pidDriveController->SetTarget(0.0, turn, relativity, this);
    m_driveControllerLog->LogPrintf("PID Turn Drive");

    return m_pidDriveController;
}

double Drive::GetPIDDistError() {
    return m_pidDriveController->GetDistError();
}

SplineDriveController *Drive::SplineDrive(TrajectoryDescription *trajectory,
                                          RelativeTo relativity) {
    this->SetDriveController(m_splineDriveController);
    m_splineDriveController->SetTarget(trajectory, relativity);
    m_driveControllerLog->LogPrintf("Spline Drive");

    return m_splineDriveController;
}

ConstantArcSplineDriveController *Drive::ConstantArcSplineDrive(
    RelativeTo relativity, double distance, double angle) {
    this->SetDriveController(m_constantArcSplineDriveController);
    m_constantArcSplineDriveController->SetTarget(relativity, distance, angle);
    m_driveControllerLog->LogPrintf("Constant Arc Spline Drive");

    return m_constantArcSplineDriveController;
}

double Drive::GetSplinePercentComplete() {
    return m_splineDriveController->GetSplinePercentComplete();
}

void Drive::SetStingerOutput(double power) {
    m_stingerDriveMotor->Set(ControlMode::PercentOutput, power);
}

void Drive::VelocityArcadeDrive(double throttle, double turn) {
    this->SetDriveController(m_velocityArcadeDriveController);
    m_velocityArcadeDriveController->SetJoysticks(throttle, turn);
    m_driveControllerLog->LogPrintf("Velocity Drive");
}

LimelightDriveController *Drive::LimelightDriveWithoutSkew() {
    this->SetDriveController(m_limelightDriveWithoutSkew);
    m_driveControllerLog->LogPrintf("Limelight No Skew Drive");

    return m_limelightDriveWithoutSkew;
}

LimelightDriveController *Drive::LimelightDriveWithSkew() {
    this->SetDriveController(m_limelightDriveWithSkew);
    m_driveControllerLog->LogPrintf("Limelight With Skew Drive");

    return m_limelightDriveWithSkew;
}

AssistedCheesyDriveController *Drive::AssistedCheesyHatchDrive(
    double throttle, double turn, bool isQuickTurn, bool isHighGear) {
    this->SetDriveController(m_assistedCheesyDriveHatchController);
    m_assistedCheesyDriveHatchController->SetJoysticks(throttle, turn,
                                                       isQuickTurn, isHighGear);
    m_driveControllerLog->LogPrintf("Assisted Cheesy Drive");

    return m_assistedCheesyDriveHatchController;
}

double Drive::GetLeftDist() const {
    return -m_leftDriveSparkA->GetEncoder().GetPosition() *
               DRIVE_DIST_PER_REVOLUTION -
           m_leftPosZero;
}

double Drive::GetRightDist() const {
    return m_rightDriveSparkA->GetEncoder().GetPosition() *
               DRIVE_DIST_PER_REVOLUTION -
           m_rightPosZero;
}

double Drive::GetLeftRate() const {
    return -m_leftDriveSparkA->GetEncoder().GetVelocity() * DRIVE_IPS_FROM_RPM;
}

double Drive::GetRightRate() const {
    return m_rightDriveSparkA->GetEncoder().GetVelocity() * DRIVE_IPS_FROM_RPM;
}

double Drive::GetDist() const {
    return (GetLeftDist() + GetRightDist()) / 2.0;
}

double Drive::GetRate() const {
    return (GetLeftRate() + GetRightRate()) / 2.0;
}

double Drive::GetDriveCurrent() const {
    return (fabs(m_rightDriveSparkA->GetOutputCurrent()) +
            fabs(m_leftDriveSparkA->GetOutputCurrent())) /
           2.0;
}

double Drive::GetAngle() const {
    return m_gyro->GetAngle();
}

double Drive::GetAngularRate() const {
    return -m_angleRate;
}

void Drive::SetDriveOutputIPS(double left, double right) {
    m_leftDriveOutput = left;
    m_rightDriveOutput = right;

    /*m_leftDriveOutput /= DRIVE_IPS_FROM_RPM;
    m_rightDriveOutput /= DRIVE_IPS_FROM_RPM;*/

    if (std::isnan(m_leftDriveOutput) || std::isnan(m_rightDriveOutput)) {
        m_leftDriveSparkA->GetPIDController().SetReference(
            0.0, ControlType::kVelocity);
        m_rightDriveSparkA->GetPIDController().SetReference(
            0.0, ControlType::kVelocity);
    }
    else {
        m_leftDriveSparkA->GetPIDController().SetReference(
            -m_leftDriveOutput, ControlType::kVelocity);
        m_rightDriveSparkA->GetPIDController().SetReference(
            m_rightDriveOutput, ControlType::kVelocity);
        DBStringPrintf(DBStringPos::DB_LINE1, "lo:%2.2lf ro:%2.2lf",
                       m_leftDriveOutput, m_rightDriveOutput);
    }
}

void Drive::SetDriveOutputPosInches(double left, double right) {
    m_leftDriveOutput = left;
    m_rightDriveOutput = right;

    m_leftDriveOutput /= DRIVE_DIST_PER_REVOLUTION;
    m_rightDriveOutput /= DRIVE_DIST_PER_REVOLUTION;

    if (std::isnan(m_leftDriveOutput) || std::isnan(m_rightDriveOutput)) {
        m_leftDriveSparkA->GetPIDController().SetReference(
            0.0, ControlType::kPosition);
        m_rightDriveSparkA->GetPIDController().SetReference(
            0.0, ControlType::kPosition);
    }
    else {
        m_leftDriveSparkA->GetPIDController().SetReference(
            -m_leftDriveOutput, ControlType::kPosition);
        m_rightDriveSparkA->GetPIDController().SetReference(
            m_rightDriveOutput, ControlType::kPosition);
    }
}

void Drive::SetDriveOutputVBus(double left, double right) {
    m_leftDriveOutput = left;
    m_rightDriveOutput = right;

    if (std::isnan(m_leftDriveOutput) || std::isnan(m_rightDriveOutput)) {
        m_leftDriveSparkA->Set(0.0);
        m_rightDriveSparkA->Set(0.0);
    }
    else {
        m_leftDriveSparkA->Set(-m_leftDriveOutput);
        m_rightDriveSparkA->Set(m_rightDriveOutput);
    }
}
void Drive::ConfigDriveCurrentLimit(double limit) {
    m_leftDriveSparkA->SetSmartCurrentLimit(limit);

    m_rightDriveSparkA->SetSmartCurrentLimit(limit);

    m_stingerDriveMotor->EnableCurrentLimit(true);
    m_stingerDriveMotor->ConfigContinuousCurrentLimit(limit, 10);
}
void Drive::DisableDriveCurrentLimit() {
    // spark can't disable?
    m_stingerDriveMotor->EnableCurrentLimit(false);
}

void Drive::EnableBrakeMode() {
    m_leftDriveSparkA->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_leftDriveSparkB->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_leftDriveSparkC->SetIdleMode(CANSparkMax::IdleMode::kBrake);

    m_rightDriveSparkA->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_rightDriveSparkB->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_rightDriveSparkC->SetIdleMode(CANSparkMax::IdleMode::kBrake);
}

void Drive::EnableCoastMode() {
    m_leftDriveSparkA->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_leftDriveSparkB->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_leftDriveSparkC->SetIdleMode(CANSparkMax::IdleMode::kCoast);

    m_rightDriveSparkA->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_rightDriveSparkB->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_rightDriveSparkC->SetIdleMode(CANSparkMax::IdleMode::kCoast);
}

void Drive::TaskPeriodic(RobotMode mode) {
    m_targetLog->LogDouble(m_limelightHatch->isTargetValid());
    m_xOffsetLog->LogDouble(m_limelightHatch->GetXOffset());
    m_yOffsetLog->LogDouble(m_limelightHatch->GetYOffset());
    m_targetAreaLog->LogDouble(m_limelightHatch->GetTargetArea());
    m_targetSkewLog->LogDouble(m_limelightHatch->GetTargetSkew());
    m_latencyLog->LogDouble(m_limelightHatch->GetLatency());
    m_pipelineLog->LogDouble(m_limelightHatch->GetPipeline());
    m_horizontalLengthLog->LogDouble(m_limelightHatch->GetHorizontalLength());
    m_verticalLengthLog->LogDouble(m_limelightHatch->GetVerticalLength());
    m_horizontalDistanceLog->LogDouble(
        m_limelightHatch->GetHorizontalDistance());

    SmartDashboard::PutNumber("drive/percentages/leftpercent",
                              m_leftDriveOutput);
    SmartDashboard::PutNumber("drive/percentages/rightpercent",
                              m_rightDriveOutput);
    SmartDashboard::PutNumber("drive/currents/leftcurrent",
                              m_leftDriveSparkA->GetOutputCurrent());
    SmartDashboard::PutNumber("drive/currents/rightcurrent",
                              m_rightDriveSparkA->GetOutputCurrent());

    // Austin ADXRS450_Gyro config
    m_angleRate = -1.0 * ((GetRightRate() - GetLeftRate()) / 2.0) /
                  (DRIVE_WIDTH / 2.0) * Constants::DEG_PER_RAD;

    /* DBStringPrintf(DB_LINE9, "l %2.1lf r %2.1lf g %2.1lf",
       this->GetLeftDist(), this->GetRightDist(), this->GetAngle()); */

    m_angleLog->LogDouble(GetAngle());
    m_angularRateLog->LogDouble(GetAngularRate());

    m_leftDistLog->LogDouble(GetLeftDist());
    m_leftDistRateLog->LogDouble(GetLeftRate());

    m_rightDistLog->LogDouble(GetRightDist());
    m_rightDistRateLog->LogDouble(GetRightRate());

    if (m_controlMode == ControlMode::Velocity) {
        m_leftDriveOutputLog->LogDouble(m_leftDriveOutput * DRIVE_IPS_FROM_RPM);
        m_rightDriveOutputLog->LogDouble(m_rightDriveOutput *
                                         DRIVE_IPS_FROM_RPM);
    }
    else if (m_controlMode == ControlMode::Position) {
        m_leftDriveOutputLog->LogDouble(m_leftDriveOutput *
                                        DRIVE_DIST_PER_REVOLUTION);
        m_rightDriveOutputLog->LogDouble(m_rightDriveOutput *
                                         DRIVE_DIST_PER_REVOLUTION);
    }
    else {
        m_leftDriveOutputLog->LogDouble(m_leftDriveOutput);
        m_rightDriveOutputLog->LogDouble(m_rightDriveOutput);
    }

    m_leftVoltageLog->LogDouble(m_leftDriveSparkA->GetAppliedOutput());
    m_rightVoltageLog->LogDouble(m_rightDriveSparkA->GetAppliedOutput());
    m_stingerVoltageLog->LogDouble(
        m_stingerDriveMotor->GetMotorOutputVoltage());

    m_currentLog->LogDouble(GetDriveCurrent());
}
}
