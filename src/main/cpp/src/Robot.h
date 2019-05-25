/**
 * 2019 Contributors:
 * - Kyle D
 * - Chris M
 * - Chris L
 * - Luis V
 * - Dylan F
 * - Allen B
 * - Andrew N
 * - John P
 */

#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/bases/CoopMTRobot.h"
#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/GreyCompressor.h"
#include "lib/helpers/GreyTalon.h"
#include "lib/helpers/GreySparkMax.h"
#include "lib/helpers/GreyLight.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/sensors/Limelight.h"
#include "src/AutonomousMode.h"
#include "src/DisabledMode.h"
#include "src/TeleopMode.h"
#include "src/TestMode.h"
#include "src/info/RobotInfo.h"
#include "src/subsystems/Drive.h"
#include "src/subsystems/Stinger.h"
#include "src/subsystems/CargoIntake.h"
#include "src/subsystems/Elevator.h"
#include "src/subsystems/HatchIntake.h"
#include <iostream>

using namespace frc;
using namespace ctre;
using namespace cs;

namespace frc973 {
/**
 * Disabled.
 */
class Disabled;
/**
 * Autonomous.
 */
class Autonomous;
/**
 * Drive.
 */
class Drive;
/**
 * Elevator.
 */
class Elevator;

/**
 * Defines the robot.
 */
class Robot
        : public CoopMTRobot
        , public DualActionJoystickObserver
        , public PoofsJoystickObserver
        , public XboxJoystickObserver {
public:
    /**
     * Constructs Robot.
     */
    Robot();
    virtual ~Robot();

    /**
     * Initializes robot.
     */
    void Initialize() override;

    /**
     * Starts disabled mode.
     */
    void DisabledStart() override;
    /**
     * Executes and repeats disabled mode procedutes.
     */
    void DisabledContinuous() override;
    /**
     * Stops disabled mode.
     */
    void DisabledStop() override;

    /**
     * Starts autonomous mode.
     */
    void AutonomousStart() override;
    /**
     * Executes and repeats autonomous mode procedures.
     */
    void AutonomousContinuous() override;
    /**
     * Stops autonomous mode.
     */
    void AutonomousStop() override;

    /**
     * Starts teleop mode.
     */
    void TeleopStart() override;
    /**
     * Executes and repeats teleop mode procedures.
     */
    void TeleopContinuous() override;
    /**
     * Stops teleop mode.
     */
    void TeleopStop() override;

    /**
     * Starts test mode
     */
    void TestStart() override;
    /**
     * Executes and repeats test mode.
     */
    void TestContinuous() override;
    /**
     * Stops test mode.
     */
    void TestStop() override;

    /**
     * Executes and repeats all modes.
     */
    void AllStateContinuous() override;

    /**
     * Observes dual action joysticks states.
     */
    void ObserveDualActionJoystickStateChange(uint32_t port, uint32_t button,
                                              bool pressedP) override;
    /**
     * Observes poofs joysticks states.
     */
    void ObservePoofsJoystickStateChange(uint32_t port, uint32_t button,
                                         bool pressedP) override;
    /**
     * Observes Xbox joystick states.
     */
    void ObserveXboxJoystickStateChange(uint32_t port, uint32_t button,
                                        bool pressedP) override;

private:
    PowerDistributionPanel *m_pdp;

    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;
    ObservableDualActionJoystick *m_testJoystick;

    LogSpreadsheet *m_logger;

    GreySparkMax *m_leftDriveSparkA;
    GreySparkMax *m_leftDriveSparkB;
    GreySparkMax *m_leftDriveSparkC;
    GreySparkMax *m_rightDriveSparkA;
    GreySparkMax *m_rightDriveSparkB;
    GreySparkMax *m_rightDriveSparkC;

    GreyTalonSRX *m_stingerDriveMotor;

    GreyTalonSRX *m_elevatorMotorA;
    VictorSPX *m_elevatorMotorB;
    DigitalInput *m_elevatorHall;

    ADXRS450_Gyro *m_gyro;

    UsbCamera m_hatchCamera;
    CameraServer *m_cameraServer;
    VideoSink m_greyCam;

    GreyTalonSRX *m_hatchRollers;
    Solenoid *m_hatchPuncher;

    Limelight *m_limelightHatch;

    GreyTalonSRX *m_cargoIntakeMotor;
    Solenoid *m_cargoWrist;
    Solenoid *m_cargoPlatformLock;

    CargoIntake *m_cargoIntake;
    HatchIntake *m_hatchIntake;
    Elevator *m_elevator;
    Drive *m_drive;
    PresetHandlerDispatcher *m_presetDispatcher;
    Stinger *m_stinger;

    DigitalInput *m_airPressureSwitch;
    Relay *m_compressorRelay;
    GreyCompressor *m_compressor;
    Teleop *m_teleop;
    Test *m_test;
    Autonomous *m_autonomous;
    Disabled *m_disabled;
};
}
