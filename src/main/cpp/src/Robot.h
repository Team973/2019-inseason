/**
 * 2019 Contributors:
 * - Kyle D
 * - Chris M
 * - Chris L
 * - Luis V
 * - Dylan F
 * - Allen B
 * - Andrew N
 **/

#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/bases/CoopMTRobot.h"
#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/GreyCompressor.h"
#include "lib/helpers/GreyTalon.h"
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
#include "src/subsystems/CargoIntake.h"
#include "src/subsystems/Elevator.h"
#include "src/subsystems/HatchIntake.h"
#include <iostream>

using namespace frc;
using namespace ctre;
using namespace cs;

namespace frc973 {
class Disabled;
class Autonomous;
class Drive;
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
    Robot();
    virtual ~Robot();

    void Initialize() override;

    void DisabledStart() override;
    void DisabledContinuous() override;
    void DisabledStop() override;

    void AutonomousStart() override;
    void AutonomousContinuous() override;
    void AutonomousStop() override;

    void TeleopStart() override;
    void TeleopContinuous() override;
    void TeleopStop() override;

    void TestStart() override;
    void TestContinuous() override;
    void TestStop() override;

    void AllStateContinuous() override;

    void ObserveDualActionJoystickStateChange(uint32_t port, uint32_t button,
                                              bool pressedP) override;
    void ObservePoofsJoystickStateChange(uint32_t port, uint32_t button,
                                         bool pressedP) override;
    void ObserveXboxJoystickStateChange(uint32_t port, uint32_t button,
                                        bool pressedP) override;

    static const int NUM_LED = 26; /**< The number of LEDs. */

private:
    PowerDistributionPanel *m_pdp;

    ObservablePoofsJoystick *m_driverJoystick;
    ObservableXboxJoystick *m_operatorJoystick;

    LogSpreadsheet *m_logger;

    GreyTalonSRX *m_leftDriveTalonA;
    VictorSPX *m_leftDriveVictorB;
    GreyTalonSRX *m_rightDriveTalonA;
    VictorSPX *m_rightDriveVictorB;

    GreyTalonSRX *m_elevatorMotorA;
    VictorSPX *m_elevatorMotorB;
    DigitalInput *m_elevatorHall;

    ADXRS450_Gyro *m_gyro;

    GreyTalonSRX *m_hatchRollers;
    Solenoid *m_hatchPuncher;

    Limelight *m_limelightCargo;
    Limelight *m_limelightHatch;

    GreyTalonSRX *m_cargoIntakeMotor;
    Solenoid *m_cargoWrist;
    Solenoid *m_cargoPlatformLock;

    GreyLight *m_greylight;

    LogCell *m_matchIdentifier;
    LogCell *m_gameSpecificMessage;

    Drive *m_drive;
    HatchIntake *m_hatchIntake;
    CargoIntake *m_cargoIntake;
    Elevator *m_elevator;

    DigitalInput *m_airPressureSwitch;
    Relay *m_compressorRelay;
    GreyCompressor *m_compressor;
    Disabled *m_disabled;
    Autonomous *m_autonomous;
    Teleop *m_teleop;
    Test *m_test;
};
}
