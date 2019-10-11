/*
 * Drive.h
 *
 *  Created on: January 7, 2019
 *      Authors: Kyle, Chris
 */
#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/helpers/GreySparkMax.h"

#include "src/controllers/CheesyDriveController.h"
#include "src/controllers/LimelightDriveController.h"
#include "src/controllers/LimelightTrigController.h"
#include "src/controllers/OpenloopArcadeDriveController.h"
#include "src/controllers/PIDDriveController.h"
#include "src/controllers/SplineDriveController.h"
#include "src/controllers/ConstantArcSplineDriveController.h"
#include "src/controllers/VelocityArcadeDriveController.h"
#include "src/controllers/AssistedCheesyDriveController.h"

namespace frc973 {

using namespace Trajectories;

/**
 * Drive provides an interface to control the drive-base (to do both
 * Teleop and Autonomous movements). To do this, it makes
 * a bunch of DriveControllers (Autonomous PID, Autonomous trap,
 * Teleop arcade, maybe someday a state space Drive controller). When
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
     * Construct a Drive.
     * @param scheduler The TaskMgr object.
     * @param logger The LogSpreadsheet object.
     * @param leftDriveSparkA The first left GreySparkMax.
     * @param leftDriveSparkB The second left GreySparkMax.
     * @param leftDriveSparkC The third left GreySparkMax.
     * @param rightDriveSparkA The first right GreySparkMax.
     * @param rightDriveSparkB The second right GreySparkMax.
     * @param rightDriveSparkC The third right GreySparkMax.
     * @param stingerDriveMotor The Stinger's GreyTalonSRX.
     * @param gyro The ADXRS450_Gyro.
     * @param limelightHatch The hatch Limelight.
     * @param hatchIntake The HatchIntake subsystem.
     * @param elevator The Elevator subsystem.
     * @param driverJoystick The driver's ObservablePoofsJoystick.
     * @param operatorJoystick The operator's ObservableXboxJoystick.
     */
    Drive(TaskMgr *scheduler, LogSpreadsheet *logger,
          GreySparkMax *leftDriveSparkA, GreySparkMax *leftDriveSparkB,
          GreySparkMax *leftDriveSparkC, GreySparkMax *rightDriveSparkA,
          GreySparkMax *rightDriveSparkB, GreySparkMax *rightDriveSparkC,
          GreyTalonSRX *stingerDriveMotor, ADXRS450_Gyro *gyro,
          Limelight *limelightHatch, HatchIntake *hatchIntake,
          Elevator *elevator, ObservablePoofsJoystick *driverJoystick,
          ObservableXboxJoystick *operatorJoystick);
    virtual ~Drive();

    /**
     * Set Drive to use the CheesyDriveController.
     * @param throttle Forward/backwards amount.
     * @param turn Left/right amount.
     * @param isQuickTurn Quickturn mode enable/disable.
     * @param isHighGear High gear enable/disable.
     */
    void CheesyDrive(double throttle, double turn, bool isQuickTurn,
                     bool isHighGear);

    /**
     * Set Drive to use the LimelightDriveController.
     * @return The LimelightDriveController.
     */
    LimelightDriveController *LimelightDrive();

    /**
     * Set Drive to use the OpenloopArcadeDriveController.
     * @param throttle Forward/backwards amount.
     * @param turn Left/right amount.
     */
    OpenloopArcadeDriveController *OpenloopArcadeDrive(double throttle,
                                                       double turn);

    /**
     * Set Drive to use the PIDDriveController to achieve a distance.
     * @param dist Distance to travel.
     * @param turn Turn value to drive with.
     * @param relativity Point RelativeTo a new setpoint.
     * @param powerCap The power cap.
     * @return The Drive controller.
     */
    PIDDriveController *PIDDrive(double dist, double turn,
                                 RelativeTo relativity, double powerCap);

    /**
     * Set Drive to use the PIDDriveController to achieve a turn.
     * @param angle Angle in degrees to go.
     * @param relativity Point RelativeTo a new setpoint.
     * @param powerCap The power cap.
     * @return The Drive controller.
     */
    PIDDriveController *PIDTurn(double angle, RelativeTo relativity,
                                double powerCap);

    /**
     * Gets the PID distance error.
     * @return The PID distance error.
     */
    double GetPIDDistError();

    /**
     * Set Drive to use the SplineDriveController.
     * @param trajectory Trajectory
     * @param relativity Point RelativeTo a new setpoint.
     * @return The SplineDriveController.
     */
    SplineDriveController *SplineDrive(TrajectoryDescription *trajectory,
                                       RelativeTo relativity);

    /**
     * Set Drive to use the ConstantArcSplineDriveController.
     * @param relativity Point RelativeTo a new setpoint
     * @param distance The distance error.
     * @param angle The angle error.
     * @return The ConstantArcSplineDriveController.
     */
    ConstantArcSplineDriveController *ConstantArcSplineDrive(
        RelativeTo relativity, double distance, double angle);

    /**
     * Gets the SplineDriveController.
     * @return The SplineDriveController.
     */
    const SplineDriveController *GetSplineDriveController() {
        return m_splineDriveController;
    }

    PIDDriveController *GetPIDDriveController() const {
        return m_pidDriveController;
    }

    /**
     * Gets percent complete of spline
     * @return The percent of the trajectory done.
     */

    double GetSplinePercentComplete();
    /**
     * Sets stinger output
     * @param power The motor power -1.0 to 1.0.
     */
    void SetStingerOutput(double power);

    /**
     * Set Drive to use the VelocityArcadeDriveController.
     * @param throttle The forward/backwards amount.
     * @param turn The left/right amount.
     */
    void VelocityArcadeDrive(double throttle, double turn);

    /**
     * Set Drive to use the LimelightDriveController with skew
     * @return The LimelightDriveController for with skew.
     */
    LimelightDriveController *LimelightDriveWithSkew();

    /**
     * Set Drive to use the LimelightDriveController without skew.
     * @return The LimelightDriveController for without skew.
     */
    LimelightDriveController *LimelightDriveWithoutSkew();

    /**
     * Get the
     * @return The LimelightDriveController for with skew.
     */
    LimelightDriveController *GetLimelightDriveWithSkew() const;

    /**
     * Set Drive to use the AssistedCheesyDriveController for Hatch.
     * @param throttle The joystick's left y-axis input.
     * @param turn The joystick's right x-axis input.
     * @param isQuickTurn The Quickturn state.
     * @param isHighGear The high gear state.
     * @return The AssistedCheesyDriveController.
     */
    AssistedCheesyDriveController *AssistedCheesyHatchDrive(double throttle,
                                                            double turn,
                                                            bool isQuickTurn,
                                                            bool isHighGear);

    /**
     * Gets the left distance from the encoder in inches.
     * @return The left distance in inches.
     */
    double GetLeftDist() const override;

    /**
     * Gets the right distance from the encoder in inches.
     * @return The right distance in inches.
     */
    double GetRightDist() const override;

    /**
     * Gets the left velocity from the encoder in inches/second.
     * @return The left velocity in inches/second.
     */
    double GetLeftRate() const override;

    /**
     * Gets the right velocity from the encoder in inches/second.
     * @return The right velocity in inches/second.
     */
    double GetRightRate() const override;

    /**
     * Gets the average distance from the encoders in inches.
     * @return The average distance.
     */
    double GetDist() const override;

    /**
     * Gets the average velocity from the encoders in inches/second.
     * @return The average velocity.
     */
    double GetRate() const override;

    /**
     * Gets the average current in amperes through Drive motor output.
     * @return The average current.
     */
    double GetDriveCurrent() const;

    /**
     * Gets the current angle from the gyro in degrees.
     * @return The current angle.
     */
    double GetAngle() const override;

    /**
     * Gets the angular rate from the gyro in degrees/second.
     * @return The angular rate.
     */
    double GetAngularRate() const override;

    /**
     * Gets the left maximum side temperature in celcius.
     * @return The left side temperature in celcius.
     */
    // double GetLeftTemperature();

    /**
     * Used by the DriveController to set motor values in inches/second.
     * @param left The velocity to send to the left motors.
     * @param right The velocity to send to the right motors.
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
     * Sets the current limit in amperes on Drive motors.
     * @param limit The current limit.
     */
    void ConfigDriveCurrentLimit(double limit);

    /**
     * Disables the current limiting on Drive motors.
     */
    void DisableDriveCurrentLimit();

    /**
     * Enables brake mode
     */
    void EnableBrakeMode();

    /**
     * Enables coast mode
     */
    void EnableCoastMode();

    /**
     * Periodically update information about the drive.
     * @param mode The current RobotMode.
     */
    void TaskPeriodic(RobotMode mode) override;

private:
    LogSpreadsheet *m_logger;

    GreySparkMax *m_leftDriveSparkA;
    GreySparkMax *m_leftDriveSparkB;
    GreySparkMax *m_leftDriveSparkC;
    GreySparkMax *m_rightDriveSparkA;
    GreySparkMax *m_rightDriveSparkB;
    GreySparkMax *m_rightDriveSparkC;

    GreyTalonSRX *m_stingerDriveMotor;
    PigeonIMU *m_pidgeygyro;
    double m_gyroAngle;

    ControlMode m_controlMode;

    double m_leftDriveOutput;
    double m_rightDriveOutput;

    LogCell *m_leftDriveOutputLog;
    LogCell *m_rightDriveOutputLog;
    LogCell *m_stingerDriveOutputLog;
    LogCell *m_leftVoltageLog;
    LogCell *m_rightVoltageLog;
    LogCell *m_stingerVoltageLog;
    LogCell *m_driveControllerLog;

    LogCell *m_targetLog;
    LogCell *m_xOffsetLog;
    LogCell *m_yOffsetLog;
    LogCell *m_targetAreaLog;
    LogCell *m_targetSkewLog;
    LogCell *m_latencyLog;
    LogCell *m_pipelineLog;
    LogCell *m_horizontalLengthLog;
    LogCell *m_verticalLengthLog;
    LogCell *m_horizontalDistanceLog;

    double m_leftPosZero;
    double m_rightPosZero;

    ADXRS450_Gyro *m_gyro;
    Limelight *m_limelightHatch;
    HatchIntake *m_hatchIntake;
    Elevator *m_elevator;
    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;

    CheesyDriveController *m_cheesyDriveController;
    OpenloopArcadeDriveController *m_openloopArcadeDriveController;
    PIDDriveController *m_pidDriveController;
    SplineDriveController *m_splineDriveController;
    ConstantArcSplineDriveController *m_constantArcSplineDriveController;
    VelocityArcadeDriveController *m_velocityArcadeDriveController;
    LimelightDriveController *m_limelightDriveWithSkew;
    LimelightDriveController *m_limelightDriveWithoutSkew;
    AssistedCheesyDriveController *m_assistedCheesyDriveHatchController;

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
