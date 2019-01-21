/*
 * Drive.h
 *
 *  Created on: January 7, 2018
 *      Authors: Kyle, Chris
 */
#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/helpers/GreyTalon.h"
#include "src/info/RobotInfo.h"
#include "lib/bases/DriveBase.h"
#include "networktables/NetworkTableInstance.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/trajectories/structs.h"

using namespace frc;
using namespace ctre;
using namespace trajectories;

namespace frc973 {
class CheesyDriveController;
class OpenloopArcadeDriveController;
class PIDDriveController;
class SplineDriveController;
class VelocityArcadeDriveController;
class LimelightDriveController;
class Limelight;
class LogSpreadsheet;

/**
 * Drive provides an interface to control the drive-base (to do both
 * teleoperated and autonomous movements).  To do this, it makes
 * a bunch of DriveControllers (autonomous PID, autonomous trap,
 * teleop arcade, maybe someday a state space drive controller).  When
 * a command is issued (one of these routines is called), Drive determines
 * which controller is best suited to service that command and makes it
 * the "active" controller.
 *
 * DriveBase: calls on the active controller to calculate motor output.
 * DriveStateProvider: provides the controller with position/angle/speed etc.
 * DrivecontrolSignalReceiver: translates controller output signal to motor
 * input signal.
 */
class Drive
        : public DriveBase
        , public DriveStateProvider
        , public DriveControlSignalReceiver {
public:
    /**
     * Construct a drive.
     * @param scheduler TaskMgr object.
     * @param logger LogSpreadsheet object.
     * @param leftDriveTalonA The first left drive motor controller.
     * @param leftDriveVictorB The second left drive motor controller.
     * @param leftDriveVictorC The third left drive motor controller.
     * @param rightDriveTalonA The first right drive motor controller.
     * @param rightDriveVictorB The second right drive motor controller.
     * @param rightDriveVictorC The third right drive motor controller.
     * @param gyro The gyro object.
     */
    Drive(TaskMgr *scheduler, LogSpreadsheet *logger,
          GreyTalonSRX *leftDriveTalonA, VictorSPX *leftDriveVictorB,
          GreyTalonSRX *rightDriveTalonA, VictorSPX *rightDriveVictorB,
          ADXRS450_Gyro *gyro, Limelight *limelight);
    virtual ~Drive();

    /**
     * Zero encoders and gyroscope.
     */
    void Zero();

    /**
     * Set a drive to use the Cheesy drive controller.
     * @param throttle Forward/backwards amount.
     * @param turn Left/right amount.
     * @param isQuickTurn Quickturn mode enable/disable.
     * @param isHighGear High gear enable/disable.
     */
    void CheesyDrive(double throttle, double turn, bool isQuickTurn,
                     bool isHighGear);

    /**
     * Set a drive to use the openloop arcade drive controller.
     * @param throttle Forward/backwards amount.
     * @param turn Left/right amount.
     */
    void OpenloopArcadeDrive(double throttle, double turn);

    /**
     * Set a drive to target a distance achieved by PID.
     * @param dist Distance to travel.
     * @param turn Turn value to drive with.
     * @param relativity Point relative to new setpoint.
     * @param powerCap The power cap.
     * @return The drive controller.
     */
    PIDDriveController *PIDDrive(double dist, double turn,
                                 RelativeTo relativity, double powerCap);

    /**
     * Set a drive to target a turn achieved by PID.
     * @param angle Angle in degrees to go.
     * @param relativity Point relative to new setpoint.
     * @param powerCap The power cap.
     * @return The drive controller.
     */
    PIDDriveController *PIDTurn(double angle, RelativeTo relativity,
                                double powerCap);

    /**
     * Return the PID distance error.
     * @return The PID distance error.
     */
    double GetPIDDistError();

    /**
     * Set a drive to use Spline drive controller.
     * @param trajectory Trajectory
     * @param relativity Point relative to new setpoint.
     * @return The drive controller.
     */
    SplineDriveController *SplineDrive(
        trajectories::TrajectoryDescription *trajectory, RelativeTo relativity);

    /**
     * Return the Spline drive controller.
     * @return The drive controller.
     */
    const SplineDriveController *GetSplineDriveController() {
        return m_splineDriveController;
    }

    /**
     * Return percent complete of spline
     * @return Percent of trajectory done.
     */
    double GetSplinePercentComplete();

    /**
     * Set drive to use the velocity arcade drive controller.
     * @param throttle Forward/backwards amount.
     * @param turn Left/right amount.
     */
    void VelocityArcadeDrive(double throttle, double turn);

    /**
     * Set drive controller to use limelight in following a target
     */
    LimelightDriveController *LimelightDrive();

    /**
     * Return the left distance from the encoder in inches.
     * @return The left distance in inches.
     */
    double GetLeftDist() const override;

    /**
     * Return the right distance from the encoder in inches.
     * @return The right distance in inches.
     */
    double GetRightDist() const override;

    /**
     * Return the left velocity from the encoder in inches/second.
     * @return The left velocity in inches/second.
     */
    double GetLeftRate() const override;

    /**
     * Return the right velocity from the encoder in inches/second.
     * @return The right velocity in inches/second.
     */
    double GetRightRate() const override;

    /**
     * Return the average distance from the encoders in inches.
     * @return The average distance.
     */
    double GetDist() const override;

    /**
     * Return the average velocity from the encoders in inches/second.
     * @return The average velocity.
     */
    double GetRate() const override;

    /**
     * Return the average current in amperes through Talon SRX output.
     * @return The average current.
     */
    double GetDriveCurrent() const;

    /**
     * Return the current angle from the gyro in degrees.
     * @return The current angle
     */
    double GetAngle() const override;

    /**
     * Return the angular rate from the gyro in degrees/second.
     * @return The angular rate.
     */
    double GetAngularRate() const override;

    /**
     * Used by the DriveController to set motor values in inches/second.
     * @param left Velocity to send to left motors.
     * @param right Velocity to send to right motors.
     */
    void SetDriveOutputIPS(double left, double right) override;

    /**
     * Used by the DriveController to set motor values in inches.
     * @param left Position to send to left motors.
     * @param right Position to send to right motors.
     */
    void SetDriveOutputPosInches(double left, double right) override;

    /**
     * Used by the DriveController to set motor values in percent.
     * @param left Percent output to send to left motors.
     * @param right Percent output to send to right motors.
     */
    void SetDriveOutputVBus(double left, double right) override;

    /**
     * Sets the current limit in amperes on drive Talon SRXs.
     * @param limit The current limit.
     */
    void ConfigDriveCurrentLimit(double limit);

    /**
     * Disables the current limiting on drive Talon SRXs.
     */
    void DisableDriveCurrentLimit();

    /**
     * Periodically update information about the drive.
     * @param mode The current robot mode.
     */
    void TaskPeriodic(RobotMode mode) override;

private:
    LogSpreadsheet *m_logger;

    GreyTalonSRX *m_leftDriveTalonA;
    VictorSPX *m_leftDriveVictorB;
    GreyTalonSRX *m_rightDriveTalonA;
    VictorSPX *m_rightDriveVictorB;

    ControlMode m_controlMode;

    double m_leftDriveOutput;
    double m_rightDriveOutput;

    LogCell *m_leftDriveOutputLog;
    LogCell *m_rightDriveOutputLog;
    LogCell *m_leftVoltageLog;
    LogCell *m_rightVoltageLog;

    double m_leftPosZero;
    double m_rightPosZero;

    ADXRS450_Gyro *m_gyro;
    double m_gyroZero;
    Limelight *m_limelight;

    CheesyDriveController *m_cheesyDriveController;
    OpenloopArcadeDriveController *m_openloopArcadeDriveController;
    PIDDriveController *m_pidDriveController;
    SplineDriveController *m_splineDriveController;
    VelocityArcadeDriveController *m_velocityArcadeDriveController;
    LimelightDriveController *m_limelightDriveController;

    double m_angle;
    double m_angleRate;
    LogCell *m_angleLog;
    LogCell *m_angularRateLog;
    LogCell *m_leftDistLog;
    LogCell *m_leftDistRateLog;
    LogCell *m_rightDistLog;
    LogCell *m_rightDistRateLog;
    LogCell *m_currentLog;
};
}
