/**
 * [year] Contributors:
 * - Person (@username)
 **/

#pragma once

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "lib/bases/CoopMTRobot.h"
#include "lib/helpers/DualActionJoystickHelper.h"
#include "lib/helpers/GreyCompressor.h"
#include "lib/helpers/GreyLight.h"
#include "lib/helpers/GreyTalon.h"
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
#include "src/subsystems/Elevator.h"
#include "src/subsystems/CargoIntake.h"
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
    VictorSPX *m_leftDriveVictorC;
    GreyTalonSRX *m_rightDriveTalonA;
    VictorSPX *m_rightDriveVictorB;
    VictorSPX *m_rightDriveVictorC;
    ADXRS450_Gyro *m_gyro;
    
    Limelight *m_limelight;
    
    GreyTalonSRX *m_elevatorMotor;

    GreyTalonSRX *m_cargoIntakeMotor;
    Solenoid *m_cargoWrist;
    Solenoid *m_cargoWristLock;
    Solenoid *m_cargoPlatformWheel;

    GreyLight *m_greylight;

    LogCell *m_matchIdentifier;
    LogCell *m_gameSpecificMessage;

    Drive *m_drive;
    Elevator *m_elevator;
    CargoIntake *m_cargoIntake;

    DigitalInput *m_airPressureSwitch;
    Relay *m_compressorRelay;
    GreyCompressor *m_compressor;
    Disabled *m_disabled;
    Autonomous *m_autonomous;
    Teleop *m_teleop;
    Test *m_test;
};
}
