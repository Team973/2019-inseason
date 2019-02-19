/*
 * RobotInfo.h
 * Created: January 9, 2018
 * Author: One of the Chris's probably
 */
#pragma once

#include "lib/util/Util.h"

using namespace frc;

namespace frc973 {
// Constants
static constexpr double DRIVE_WIDTH = 24.0;
static constexpr double DRIVE_WHEEL_DIAMETER = 4.0;
static constexpr double DRIVE_ENCODER_COUNT = 4096;
static constexpr double DRIVE_DIST_PER_REVOLUTION =
    DRIVE_WHEEL_DIAMETER * Constants::PI;
static constexpr double DRIVE_DIST_PER_CLICK =
    DRIVE_DIST_PER_REVOLUTION / DRIVE_ENCODER_COUNT;
static constexpr double DRIVE_IPS_FROM_RPM = DRIVE_DIST_PER_REVOLUTION / 60.0;
static constexpr double DRIVE_IPS_FROM_CPDS =
    1 * 10 / DRIVE_ENCODER_COUNT * DRIVE_DIST_PER_REVOLUTION;  // aka 0.0383

/**
 * Pneumatics Hardware
 */
static const int PCM_CAN_ID = 17;
static const int COMPRESSOR_RELAY = 0;

/**
 * Drive Subsystem
 **/
static const int RIGHT_DRIVE_A_CAN_ID = 1;
static const int RIGHT_DRIVE_B_VICTOR_ID = 2;
static const int LEFT_DRIVE_A_CAN_ID = 16;
static const int LEFT_DRIVE_B_VICTOR_ID = 15;

/**
 * Elevator Subsystem
 */
static const int ELEVATOR_A_CAN_ID = 3;
static const int ELEVATOR_B_CAN_ID = 4;

/**
 * Intake Subsystem
 **/
static const int CARGO_INTAKE_CAN_ID = 5;
static const int CARGO_INTAKE_WRIST_PCM_ID = 1;
static const int CARGO_PLATFORM_LOCK_PCM_ID = 0;
static const int INTAKE_CAN_ID = 4;
static const int HATCH_PUNCHER_PCM_ID = 2;
static const int HATCH_ROLLER_CAN_ID = 14;
static const int LEFT_HATCH_SENSOR_ID = 2;
static const int RIGHT_HATCH_SENSOR_ID = 3;

/**
 * Stinger Subsystem
 */
static const int STINGER_ELEVATOR_CAN_ID = 12;
static const int STINGER_DRIVE_CAN_ID = 13;
// note: stinger drive motor is in drive subsystem
static const int STINGER_LOWER_HALL_DIN_ID = 8;
static const int STINGER_UPPER_HALL_DIN_ID = 9;

/**
 * Digital Sensors
 **/
static const int PRESSURE_DIN_ID = 1;
static const int ELEVATOR_HALL_ID = 7;

/**
 * Analog Sensors
 **/

/**
 * Joysticks
 */
static const int DRIVER_JOYSTICK_PORT = 0;
static const int OPERATOR_JOYSTICK_PORT = 1;
static const int DRIVER_DA_JOYSTICK_PORT = 2;
static const int OPERATOR_DA_JOYSTICK_PORT = 3;
}
