#pragma once

#include "lib/bases/CoopMTRobot.h"
#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/GreyCompressor.h"
#include "lib/helpers/GreyCTRE.h"
#include "lib/helpers/GreySparkMax.h"
#include "lib/helpers/PoofsJoystickHelper.h"
#include "lib/helpers/XboxJoystickHelper.h"
#include "lib/logging/LogSpreadsheet.h"
#include "lib/sensors/Limelight.h"

#include "src/info/RobotInfo.h"
#include "src/subsystems/Drive.h"
#include "src/subsystems/Stinger.h"
#include "src/subsystems/CargoIntake.h"
#include "src/subsystems/Elevator.h"
#include "src/subsystems/HatchIntake.h"
#include "src/AutonomousMode.h"
#include "src/DisabledMode.h"
#include "src/TeleopMode.h"
#include "src/TestMode.h"

using namespace cs;

/**
 * Main Robot Code for FRC Team 973.
 * @author Kyle D
 * @author Chris M
 * @author Chris L
 * @author Luis V
 * @author Dylan F
 * @author Allen B
 * @author Andrew
 * @author John P
 */
namespace frc973 {

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
     * Starts Disabled mode.
     */
    void DisabledStart() override;

    /**
     * Executes and repeats Disabled mode procedutes.
     */
    void DisabledContinuous() override;

    /**
     * Stops Disabled mode.
     */
    void DisabledStop() override;

    /**
     * Starts Autonomous mode.
     */
    void AutonomousStart() override;

    /**
     * Executes and repeats Autonomous mode procedures.
     */
    void AutonomousContinuous() override;

    /**
     * Stops Autonomous mode.
     */
    void AutonomousStop() override;

    /**
     * Starts Teleop mode.
     */
    void TeleopStart() override;
    /**
     * Executes and repeats Teleop mode procedures.
     */
    void TeleopContinuous() override;
    /**
     * Stops Teleop mode.
     */
    void TeleopStop() override;

    /**
     * Starts Test mode
     */
    void TestStart() override;
    /**
     * Executes and repeats Test mode.
     */
    void TestContinuous() override;
    /**
     * Stops Test mode.
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
     * Observes PoofsJoysticks states.
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
    ObservableDualActionJoystick *m_tuningJoystick;

    LogSpreadsheet *m_logger;

    GreySparkMax *m_leftDriveSparkA;
    GreySparkMax *m_leftDriveSparkB;
    GreySparkMax *m_leftDriveSparkC;
    GreySparkMax *m_rightDriveSparkA;
    GreySparkMax *m_rightDriveSparkB;
    GreySparkMax *m_rightDriveSparkC;

    GreyTalonSRX *m_stingerDriveMotor;

    GreyTalonSRX *m_elevatorMotorA;
    GreyVictorSPX *m_elevatorMotorB;
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
