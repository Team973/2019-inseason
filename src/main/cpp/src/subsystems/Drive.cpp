/*
 * Drive.cpp
 *
 *  Created on: January 7, 2019
 *      Authors: Luis and Dylan
 */

#include <stdio.h>
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "src/controllers/CheesyDriveController.h"
#include "src/controllers/LimelightDriveController.h"
#include "src/controllers/OpenloopArcadeDriveController.h"
#include "src/controllers/PIDDriveController.h"
#include "src/controllers/SplineDriveController.h"
#include "src/controllers/VelocityArcadeDriveController.h"
#include "src/controllers/LimelightDriveController.h"
#include "src/controllers/AssistedCheesyDriveController.h"
#include "src/info/RobotInfo.h"
#include "src/subsystems/Drive.h"
#include "lib/util/Util.h"
#include "lib/util/WrapDash.h"
#include "lib/sensors/Limelight.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/trajectories/structs.h"

using namespace frc;
using namespace ctre;
using namespace rev;
using namespace trajectories;

namespace frc973 {
Drive::Drive(TaskMgr *scheduler, LogSpreadsheet *logger,
             GreySparkMax *leftDriveSparkA, GreySparkMax *leftDriveSparkB,
             GreySparkMax *rightDriveSparkA, GreySparkMax *rightDriveSparkB,
             GreyTalonSRX *stingerDriveMotor, ADXRS450_Gyro *gyro,
             Limelight *limelightHatch)
        : DriveBase(scheduler, this, this, nullptr)
        , m_logger(logger)
        , m_leftDriveSparkA(leftDriveSparkA)
        , m_leftDriveSparkB(leftDriveSparkB)
        , m_rightDriveSparkA(rightDriveSparkA)
        , m_rightDriveSparkB(rightDriveSparkB)
        , m_stingerDriveMotor(stingerDriveMotor)
        , m_controlMode(ControlMode::PercentOutput)
        , m_leftDriveOutput(0.0)
        , m_rightDriveOutput(0.0)
        , m_leftDriveOutputLog(new LogCell("Left motor signal (pow or vel)"))
        , m_rightDriveOutputLog(new LogCell("Right motor signal (pow or vel)"))
        , m_stingerDriveOutputLog(
              new LogCell("Stinger motor signal (pow or vel)"))
        , m_leftVoltageLog(new LogCell("Left motor voltage"))
        , m_rightVoltageLog(new LogCell("Right motor voltage"))
        , m_stingerVoltageLog(new LogCell("Stinger motor voltage"))
        , m_leftPosZero(0.0)
        , m_rightPosZero(0.0)
        , m_gyro(gyro)
        , m_gyroZero(0.0)
        , m_limelightHatch(limelightHatch)
        , m_cheesyDriveController(new CheesyDriveController(limelightHatch))
        , m_openloopArcadeDriveController(new OpenloopArcadeDriveController())
        , m_pidDriveController(new PIDDriveController())
        , m_splineDriveController(new SplineDriveController(this, logger))
        , m_velocityArcadeDriveController(new VelocityArcadeDriveController())
        , m_limelightHatchDriveController(new LimelightDriveController(
              limelightHatch, LimelightDriveController::VisionOffset::Hatch,
              false))
        , m_regularLimelightHatchDriveController(new LimelightDriveController(
              limelightHatch, LimelightDriveController::VisionOffset::Hatch,
              true))
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
    m_leftDriveSparkA->Config_PID(0, 0.0, 0.0, 0.0, 0.0);

    m_leftDriveSparkB->Follow(*m_leftDriveSparkA);
    m_leftDriveSparkB->SetInverted(false);

    m_rightDriveSparkA->SetIdleMode(CANSparkMax::IdleMode::kCoast);
    m_rightDriveSparkA->SetInverted(false);
    m_rightDriveSparkA->Config_PID(0, 0.0, 0.0, 0.0, 0.0);

    m_rightDriveSparkB->Follow(*m_rightDriveSparkA);
    m_rightDriveSparkB->SetInverted(false);

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
}

Drive::~Drive() {
}

void Drive::CheesyDrive(double throttle, double turn, bool isQuickTurn,
                        bool isHighGear) {
    this->SetDriveController(m_cheesyDriveController);
    m_cheesyDriveController->SetJoysticks(throttle, turn, isQuickTurn,
                                          isHighGear);
}

LimelightDriveController *Drive::LimelightDrive() {
    this->SetDriveController(m_limelightDriveController);

    return m_limelightDriveController;
}

void Drive::OpenloopArcadeDrive(double throttle, double turn) {
    this->SetDriveController(m_openloopArcadeDriveController);
    m_openloopArcadeDriveController->SetJoysticks(throttle, turn);
}

PIDDriveController *Drive::PIDDrive(double dist, double turn,
                                    RelativeTo relativity, double powerCap) {
    this->SetDriveController(m_pidDriveController);
    m_pidDriveController->SetCap(powerCap);
    m_pidDriveController->SetTarget(dist, turn, relativity, this);
    return m_pidDriveController;
}

PIDDriveController *Drive::PIDTurn(double turn, RelativeTo relativity,
                                   double powerCap) {
    this->SetDriveController(m_pidDriveController);
    m_pidDriveController->SetCap(powerCap);
    m_pidDriveController->SetTarget(0.0, turn, relativity, this);
    return m_pidDriveController;
}

double Drive::GetPIDDistError() {
    return m_pidDriveController->GetDistError();
}

SplineDriveController *Drive::SplineDrive(
    trajectories::TrajectoryDescription *trajectory, RelativeTo relativity) {
    this->SetDriveController(m_splineDriveController);
    m_splineDriveController->SetTarget(trajectory, relativity);
    return m_splineDriveController;
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
}

LimelightDriveController *Drive::LimelightHatchDrive() {
    this->SetDriveController(m_limelightHatchDriveController);

    return m_limelightHatchDriveController;
}

LimelightDriveController *Drive::RegularLimelightHatchDrive() {
    this->SetDriveController(m_regularLimelightHatchDriveController);

    return m_limelightHatchDriveController;
}

AssistedCheesyDriveController *Drive::AssistedCheesyHatchDrive(
    double throttle, double turn, bool isQuickTurn, bool isHighGear) {
    this->SetDriveController(m_assistedCheesyDriveHatchController);
    m_assistedCheesyDriveHatchController->SetJoysticks(throttle, turn,
                                                       isQuickTurn, isHighGear);
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
    return -(m_angle - m_gyroZero);
}

double Drive::GetAngularRate() const {
    return -m_angleRate;
}

void Drive::SetDriveOutputIPS(double left, double right) {
    m_leftDriveOutput = left;
    m_rightDriveOutput = right;

    m_leftDriveOutput /= DRIVE_IPS_FROM_RPM;
    m_rightDriveOutput /= DRIVE_IPS_FROM_RPM;

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

void Drive::TaskPeriodic(RobotMode mode) {
    m_angle = m_gyro->GetAngle();

    // Austin ADXRS450_Gyro config
    m_angleRate = -1.0 * ((GetRightRate() - GetLeftRate()) / 2.0) /
                  (DRIVE_WIDTH / 2.0) * Constants::DEG_PER_RAD;

    DBStringPrintf(DB_LINE9, "l %2.1lf r %2.1lf g %2.1lf", this->GetLeftDist(),
                   this->GetRightDist(), this->GetAngle());

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
