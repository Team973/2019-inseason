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
using namespace trajectories;

namespace frc973 {
Drive::Drive(TaskMgr *scheduler, LogSpreadsheet *logger,
             GreyTalonSRX *leftDriveTalonA, VictorSPX *leftDriveVictorB,
             VictorSPX *leftDriveVictorC, GreyTalonSRX *rightDriveTalonA,
             VictorSPX *rightDriveVictorB, VictorSPX *rightDriveVictorC,
             ADXRS450_Gyro *gyro, Limelight *limelightCargo,
             Limelight *limelightHatch)
        : DriveBase(scheduler, this, this, nullptr)
        , m_logger(logger)
        , m_leftDriveTalonA(leftDriveTalonA)
        , m_leftDriveVictorB(leftDriveVictorB)
        , m_leftDriveVictorC(leftDriveVictorC)
        , m_rightDriveTalonA(rightDriveTalonA)
        , m_rightDriveVictorB(rightDriveVictorB)
        , m_rightDriveVictorC(rightDriveVictorC)
        , m_controlMode(ControlMode::PercentOutput)
        , m_leftDriveOutput(0.0)
        , m_rightDriveOutput(0.0)
        , m_leftDriveOutputLog(new LogCell("Left motor signal (pow or vel)"))
        , m_rightDriveOutputLog(new LogCell("Right motor signal (pow or vel)"))
        , m_leftVoltageLog(new LogCell("Left motor voltage"))
        , m_rightVoltageLog(new LogCell("Right motor voltage"))
        , m_leftPosZero(0.0)
        , m_rightPosZero(0.0)
        , m_gyro(gyro)
        , m_gyroZero(0.0)
        , m_limelightCargo(limelightCargo)
        , m_limelightHatch(limelightHatch)
        , m_cheesyDriveController(
              new CheesyDriveController(limelightCargo, limelightHatch))
        , m_openloopArcadeDriveController(new OpenloopArcadeDriveController())
        , m_pidDriveController(new PIDDriveController())
        , m_splineDriveController(new SplineDriveController(this, logger))
        , m_velocityArcadeDriveController(new VelocityArcadeDriveController())
        , m_limelightCargoDriveController(
              new LimelightDriveController(limelightCargo))
        , m_limelightHatchDriveController(
              new LimelightDriveController(limelightHatch))
        , m_assistedCheesyDriveController(
              new AssistedCheesyDriveController(m_limelightHatch))
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
    m_leftDriveTalonA->SetNeutralMode(Coast);
    m_leftDriveTalonA->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 10);
    m_leftDriveTalonA->SetSensorPhase(false);
    m_leftDriveTalonA->SetInverted(false);
    m_leftDriveTalonA->SelectProfileSlot(0, 0);
    m_leftDriveTalonA->Config_kP(0, 0.5, 0);
    m_leftDriveTalonA->Config_kI(0, 0.0, 0);
    m_leftDriveTalonA->Config_kD(0, 0.0, 0);
    m_leftDriveTalonA->Config_kF(0, 0.0, 0);

    m_leftDriveVictorB->Follow(*m_leftDriveTalonA);
    m_leftDriveVictorB->SetInverted(false);

    m_leftDriveVictorC->Follow(*m_leftDriveTalonA);
    m_leftDriveVictorC->SetInverted(false);

    m_rightDriveTalonA->SetNeutralMode(Coast);
    m_rightDriveTalonA->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 10);
    m_rightDriveTalonA->SetSensorPhase(false);
    m_rightDriveTalonA->SetInverted(false);
    m_rightDriveTalonA->SelectProfileSlot(0, 0);
    m_rightDriveTalonA->Config_kP(0, 0.5, 0);
    m_rightDriveTalonA->Config_kI(0, 0.0, 0);
    m_rightDriveTalonA->Config_kD(0, 0.0, 0);
    m_rightDriveTalonA->Config_kF(0, 0.0, 0);

    m_rightDriveVictorB->Follow(*m_rightDriveTalonA);
    m_rightDriveVictorB->SetInverted(false);

    m_rightDriveVictorC->Follow(*m_rightDriveTalonA);
    m_rightDriveVictorC->SetInverted(false);

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

void Drive::VelocityArcadeDrive(double throttle, double turn) {
    this->SetDriveController(m_velocityArcadeDriveController);
    m_velocityArcadeDriveController->SetJoysticks(throttle, turn);
}

LimelightDriveController *Drive::LimelightCargoDrive() {
    this->SetDriveController(m_limelightCargoDriveController);

    return m_limelightCargoDriveController;
}

LimelightDriveController *Drive::LimelightHatchDrive() {
    this->SetDriveController(m_limelightHatchDriveController);

    return m_limelightHatchDriveController;
}

AssistedCheesyDriveController *Drive::AssistedCheesyDrive() {
    this->SetDriveController(m_assistedCheesyDriveController);
    return m_assistedCheesyDriveController;
}

double Drive::GetLeftDist() const {
    return -m_leftDriveTalonA->GetSelectedSensorPosition(0) *
               DRIVE_DIST_PER_CLICK -
           m_leftPosZero;
}

double Drive::GetRightDist() const {
    return m_rightDriveTalonA->GetSelectedSensorPosition(0) *
               DRIVE_DIST_PER_CLICK -
           m_rightPosZero;
}

double Drive::GetLeftRate() const {
    return -m_leftDriveTalonA->GetSelectedSensorVelocity(0) *
           DRIVE_IPS_FROM_CPDS;
}

double Drive::GetRightRate() const {
    return m_rightDriveTalonA->GetSelectedSensorVelocity(0) *
           DRIVE_IPS_FROM_CPDS;
}

double Drive::GetDist() const {
    return (GetLeftDist() + GetRightDist()) / 2.0;
}

double Drive::GetRate() const {
    return (GetLeftRate() + GetRightRate()) / 2.0;
}

double Drive::GetDriveCurrent() const {
    return (fabs(m_rightDriveTalonA->GetOutputCurrent()) +
            fabs(m_leftDriveTalonA->GetOutputCurrent())) /
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

    m_leftDriveOutput /= DRIVE_IPS_FROM_CPDS;
    m_rightDriveOutput /= DRIVE_IPS_FROM_CPDS;

    if (std::isnan(m_leftDriveOutput) || std::isnan(m_rightDriveOutput)) {
        m_leftDriveTalonA->Set(ControlMode::Velocity, 0.0);
        m_rightDriveTalonA->Set(ControlMode::Velocity, 0.0);
    }
    else {
        m_leftDriveTalonA->Set(ControlMode::Velocity, -m_leftDriveOutput);
        m_rightDriveTalonA->Set(ControlMode::Velocity, m_rightDriveOutput);
    }
}

void Drive::SetDriveOutputPosInches(double left, double right) {
    m_leftDriveOutput = left;
    m_rightDriveOutput = right;

    m_leftDriveOutput /= DRIVE_DIST_PER_CLICK;
    m_rightDriveOutput /= DRIVE_DIST_PER_CLICK;

    if (std::isnan(m_leftDriveOutput) || std::isnan(m_rightDriveOutput)) {
        m_leftDriveTalonA->Set(ControlMode::Position, 0.0);
        m_rightDriveTalonA->Set(ControlMode::Position, 0.0);
    }
    else {
        m_leftDriveTalonA->Set(ControlMode::Position, -m_leftDriveOutput);
        m_rightDriveTalonA->Set(ControlMode::Position, m_rightDriveOutput);
    }
}

void Drive::SetDriveOutputVBus(double left, double right) {
    m_leftDriveOutput = left;
    m_rightDriveOutput = right;

    if (std::isnan(m_leftDriveOutput) || std::isnan(m_rightDriveOutput)) {
        m_leftDriveTalonA->Set(ControlMode::PercentOutput, 0.0);
        m_rightDriveTalonA->Set(ControlMode::PercentOutput, 0.0);
    }
    else {
        m_leftDriveTalonA->Set(ControlMode::PercentOutput, -m_leftDriveOutput);
        m_rightDriveTalonA->Set(ControlMode::PercentOutput, m_rightDriveOutput);
    }
}
void Drive::ConfigDriveCurrentLimit(double limit) {
    m_leftDriveTalonA->EnableCurrentLimit(true);
    m_leftDriveTalonA->ConfigContinuousCurrentLimit(limit, 10);

    m_rightDriveTalonA->EnableCurrentLimit(true);
    m_rightDriveTalonA->ConfigContinuousCurrentLimit(limit, 10);
}
void Drive::DisableDriveCurrentLimit() {
    m_leftDriveTalonA->EnableCurrentLimit(false);

    m_rightDriveTalonA->EnableCurrentLimit(false);
}

void Drive::TaskPeriodic(RobotMode mode) {
    // NetworkTable Voltages
    SmartDashboard::PutNumber("drive/voltages/leftvoltage",
                              m_leftDriveTalonA->GetBusVoltage() / 12.0);
    SmartDashboard::PutNumber("drive/voltages/rightvoltage",
                              m_rightDriveTalonA->GetBusVoltage() / 12.0);

    // NetworkTable Currents
    SmartDashboard::PutNumber("drive/currents/leftcurrent",
                              m_leftDriveTalonA->GetOutputCurrent());
    SmartDashboard::PutNumber("drive/currents/rightcurrent",
                              m_rightDriveTalonA->GetOutputCurrent());

    // NetworkTable Encoders
    SmartDashboard::PutNumber("drive/encoders/leftencoder", GetLeftDist());
    SmartDashboard::PutNumber("drive/encoders/rightencoder", GetRightDist());

    // NetworkTable motor output
    SmartDashboard::PutNumber("drive/outputs/leftratesetpoint",
                              m_leftDriveOutput * DRIVE_IPS_FROM_CPDS);
    SmartDashboard::PutNumber("drive/outputs/leftrateactual",
                              Drive::GetLeftRate());

    SmartDashboard::PutNumber("drive/outputs/rightratesetpoint",
                              m_rightDriveOutput * DRIVE_IPS_FROM_CPDS);
    SmartDashboard::PutNumber("drive/outputs/rightrateactual",
                              Drive::GetRightRate());

    // NetworkTable Limelight
    SmartDashboard::PutNumber("drive/limelight/error",
                              m_limelightHatch->GetXOffset());

    // NetworkTable Gyro
    SmartDashboard::PutNumber("drive/gyro/angle", this->GetAngle());

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
        m_leftDriveOutputLog->LogDouble(m_leftDriveOutput *
                                        DRIVE_IPS_FROM_CPDS);
        m_rightDriveOutputLog->LogDouble(m_rightDriveOutput *
                                         DRIVE_IPS_FROM_CPDS);
    }
    else if (m_controlMode == ControlMode::Position) {
        m_leftDriveOutputLog->LogDouble(m_leftDriveOutput *
                                        DRIVE_DIST_PER_CLICK);
        m_rightDriveOutputLog->LogDouble(m_rightDriveOutput *
                                         DRIVE_DIST_PER_CLICK);
    }
    else {
        m_leftDriveOutputLog->LogDouble(m_leftDriveOutput);
        m_rightDriveOutputLog->LogDouble(m_rightDriveOutput);
    }

    m_leftVoltageLog->LogDouble(m_leftDriveTalonA->GetMotorOutputVoltage());
    m_rightVoltageLog->LogDouble(m_rightDriveTalonA->GetMotorOutputVoltage());

    m_currentLog->LogDouble(GetDriveCurrent());
}
}
