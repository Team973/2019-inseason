/*
 * CoopMTRobot.h
 *
 *  Created on: Sep 1, 2015
 *      Author: Andrew
 *
 * Functionality pretty similar to WPILIB's Iterative Robot
 * except that this one manages tasks and has a Start and Stop
 * method for each state.
 *
 * A more ideal solution would probably have been to inherit from
 * SimpleRobot and use the same virtual method names as Iterative Robot,
 * but TimedRobot handles driver station and HAL communication and those
 * protocols are subject to change... this should be more maintainable
 * in the long run.
 */

#pragma once

#include "stdint.h"
#include "frc/WPILib.h"
#include "lib/managers/TaskMgr.h"
#include "lib/util/Util.h"
#include <pthread.h>

#ifndef PROGRAM_NAME
#define PROGRAM_NAME "(unspecified)"
#endif

using namespace frc;

namespace frc973 {

static constexpr int MAXHOSTNAMELEN = 128;

/**
 * CoopMTRobot extends the TimedRobot robot program framework. It is an
 * interface for a cooperative multitasking robot that is managed and kept in
 * sync with other threads. Each task returns after some period of time. The
 * CoopMTRobot class is intended to be subclassed by a user creating a robot
 * program. In addition to Continuous() functions from this class being called
 * each time a new packet is received from the driver station, tasks registered
 * with this class (via the TaskMgr interface) are also called each time a new
 * packet is received from the driver station.
 */
class CoopMTRobot
        : public TimedRobot
        , public TaskMgr
        , public frc::RobotState {
public:
    /**
     * Constuct a Coop MT Robot.
     */
    CoopMTRobot();
    virtual ~CoopMTRobot();

    /**
     * Similar to RobotInit in the TimedRobot class, override Initialize
     * to do any post-constructor initialization.
     */
    virtual void Initialize() {
    }

    /**
     * The following {Disabled,Autonomous,Teleop,Test}{Start,Stop,Continuous}
     * should be overridden to get behavior similar to overriding those
     * functions in Iterative Robot:
     *   - [Mode]Start is called when robot first changes into this mode (like
     * [Mode]Init in TimedRobot).
     *   - [Mode]Stop is called when robot moves to another mode from [Mode].
     *   - [Mode]Continuous is called repeatedly every 20ms (like [Mode]Periodic
     * in TimedRobot).
     *   - [Mode]Start is garaunteed to be called before [Mode]Continuous.
     *   - [Mode]Stop for the previous mode is garaunteed to be called before
     * [Mode]Start from the prevous mode.
     */

    /**
     * DisabledStart is called when robot first changes into this mode (like
     * DisabledInit in TimedRobot). DisabledStart is garaunteed to be called
     * before DisabledContinuous.
     */
    virtual void DisabledStart() {
    }

    /**
     * DisabledStop is called when robot moves to another mode from Disabled.
     * DisabledStop for the previous mode is garaunteed to be called before
     * [Mode]Start from the prevous mode.
     */
    virtual void DisabledStop() {
    }

    /**
     * DisabledContinuous is called repeatedly every 20ms (like DisabledPeriodic
     * in TimedRobot).
     */
    virtual void DisabledContinuous() {
    }

    /**
     * AutonomousStart is called when robot first changes into this mode (like
     * AutonomousInit in TimedRobot). AutonomousStart is garaunteed to be
     * called before AutonomousContinuous.
     */
    virtual void AutonomousStart() {
    }

    /**
     * AutonomousStop is called when robot moves to another mode from
     * Autonomous. AutonomousStop for the previous mode is garaunteed to be
     * called before [Mode]Start from the prevous mode.
     */
    virtual void AutonomousStop() {
    }

    /**
     * AutonomousContinuous is called repeatedly every 20ms (like
     * AutonomousPeriodic in TimedRobot).
     */
    virtual void AutonomousContinuous() {
    }

    /**
     * TelopStart is called when robot first changes into this mode (like
     * TelopInit in TimedRobot). TelopStart is garaunteed to be called
     * before TelopContinuous.
     */
    virtual void TeleopStart() {
    }

    /**
     * TeleopStop is called when robot moves to another mode from Teleop.
     * TeleopStop for the previous mode is garaunteed to be called before
     * [Mode]Start from the prevous mode.
     */
    virtual void TeleopStop() {
    }

    /**
     * TeleopContinuous is called repeatedly every 20ms (like TeleopPeriodic in
     * TimedRobot).
     */
    virtual void TeleopContinuous() {
    }

    /**
     * TestStart is called when robot first changes into this mode (like
     * TestInit in TimedRobot). TestStart is garaunteed to be called before
     * TestContinuous.
     */
    virtual void TestStart() {
    }

    /**
     * TestStop is called when robot moves to another mode from Test. TestStop
     * for the previous mode is garaunteed to be called before [Mode]Start from
     * the prevous mode.
     */
    virtual void TestStop() {
    }

    /**
     * TestContinuous is called repeatedly every 20ms (like TestPeriodic in
     * TimedRobot).
     */
    virtual void TestContinuous() {
    }

    /**
     * Called continuously during all robot stages
     */
    virtual void AllStateContinuous() {
    }

protected:
    /**
     * For internal use only.  Children of this object should not try to
     * override these (if they do, they *WILL NOT GET RUN*).
     */

    /**
     * Start the robot program. Don't implement this in any child classes.
     */
    void RobotInit() override;

    /**
     * Stop the previous mode and start Disabled. Don't implement this in any
     * child classes.
     */
    void DisabledInit() override;

    /**
     * Stop the previous mode and start Autonomous. Don't implement this in any
     * child classes.
     */
    void AutonomousInit() override;

    /**
     * Stop the previous mode and start Teleop. Don't implement this in any
     * child classes.
     */
    void TeleopInit() override;

    /**
     * Stop the previous mode and start Test. Don't implement this in any child
     * classes.
     */
    void TestInit() override;

    /**
     * Start the Disabled loop. Don't implement this in any child classes.
     */
    void DisabledPeriodic() override;

    /**
     * Start the Autonomous loop. Don't implement this in any child classes.
     */
    void AutonomousPeriodic() override;

    /**
     * Start the Teleop loop. Don't implement this in any child classes.
     */
    void TeleopPeriodic() override;

    /**
     * Start the Test loop. Don't implement this in any child classes.
     */
    void TestPeriodic() override;

    /**
     * Stop a robot mode. Don't implement this in any child classes.
     * @param toStop The robot mode to stop.
     */
    void ModeStop(RobotMode toStop);

    /**
     * Start a robot mode. Don't implement this in any child classes.
     * @param toStart The robot mode to start.
     */
    void ModeStart(RobotMode toStart);

    /**
     * Return disabled status. Don't implement this in any child classes.
     * @return Whether the robot is disabled or not.
     */
    bool IsDisabled() const;

    /**
     * Return enabled status. Don't implement this in any child classes.
     * @return Whether the robot is enabled or not.
     */
    bool IsEnabled() const;

    /**
     * Return teleop status. Don't implement this in any child classes.
     * @return Whether the robot is enabled in teleop or not.
     */
    bool IsOperatorControl() const;

    /**
     * Return autonomous status. Don't implement this in any child classes.
     * @return Whether the robot is enabled in autonomous or not.
     */
    bool IsAutonomous() const;

    /**
     * Return test status. Don't implement this in any child classes.
     * @return Whether the robot is enabled in test or not.
     */
    bool IsTest() const;

private:
    RobotMode m_prevMode;
    mutable pthread_mutex_t m_robotModeMutex;
};
}
