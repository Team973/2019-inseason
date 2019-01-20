#include "src/AutonomousMode.h"
#include "src/DisabledMode.h"
#include "src/Robot.h"

using namespace frc;

namespace frc973 {
Autonomous::Autonomous(Disabled *disabled, Drive *drive, ADXRS450_Gyro *gyro,
                       ObservablePoofsJoystick *driver,
                       ObservableXboxJoystick *codriver)
        : m_driverJoystick(driver)
        , m_operatorJoystick(codriver)
        , m_driveMode(DriveMode::Cheesy)
        , m_endGameSignalSent(false)
        , m_noAuto(new NoAuto())
        , m_forwardAuto(new ForwardAuto(drive))
        , m_disabled(disabled)
        , m_drive(drive)
        , m_gyro(gyro)
        , m_autoSignal(new LightPattern::AutoIndicator()) {
}

Autonomous::~Autonomous() {
}

void Autonomous::AutonomousInit() {
    // Remember to zero all sensors here
    m_gyro->Reset();
    std::cout << "Autonomous Start" << std::endl;

    m_forwardAuto->Reset();
    m_routine = m_forwardAuto;
}

void Autonomous::AutonomousPeriodic() {
    /**
     * Driver Joystick
     */
    double y =
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::LeftYAxis);
    double x =
        -m_driverJoystick->GetRawAxisWithDeadband(PoofsJoysticks::RightXAxis);
    bool quickturn =
        m_driverJoystick->GetRawButton(PoofsJoysticks::RightBumper);
    bool lowGear = m_driverJoystick->GetRawButton(PoofsJoysticks::RightTrigger);

    if (m_driveMode == DriveMode::Cheesy) {
        if (lowGear) {
            m_drive->CheesyDrive(y / 3.0, x / 3.0, quickturn, false);
        }
        else {
            m_drive->CheesyDrive(y, x, quickturn, false);
        }
    }
    else if (m_driveMode == DriveMode::Openloop) {
        if (lowGear) {
            m_drive->OpenloopArcadeDrive(y / 3.0, x / 3.0);
        }
        else {
            m_drive->OpenloopArcadeDrive(y, x);
        }
    }

    /**
     * Operator Joystick
     */
}

void Autonomous::AutonomousStop() {
}
}
