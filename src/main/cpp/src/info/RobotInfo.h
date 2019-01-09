/*
 * RobotInfo.h
 * Created: January 9, 2018
 * Author: One of the Chris's probably
 */
#pragma once

#include "lib/util/Util.h"

using namespace frc;

namespace frc973 {

/**
 * Arm Subsystem
 */

// CAN IDs
static const int ARM_CAN_ID = 4;
static const int ARM_MAX_HALL_DIN = 3;
static const int ARM_MIN_HALL_DIN = 4;
static const int ARM_ENCODER_COUNT = 4096.0;
static const double ARM_ENCODER_DEGREES_PER_CLICK = 360.0 / ARM_ENCODER_COUNT;
static const double ARM_GEAR_RATIO =
    9.0 / 54.0;  // arm revolutions per encoder revolutions
static const double ARM_DEGREES_PER_CLICK =
    ARM_GEAR_RATIO * ARM_ENCODER_DEGREES_PER_CLICK;

/**
 * Claw Subsystem
 */

// CAN IDs
static const int CLAW_LEFT_ROLLER_CAN_ID = 12;
static const int CLAW_RIGHT_ROLLER_CAN_ID = 5;

// Digital Inputs
static const int CUBE_TIME_OF_FLIGHT_DIN = 2;

// Pneumatics
static const int HATCH_PUNCHER_PCM_ID = 0;
static const int CUBE_CLAMP_PCM_ID = 1;
static const int CUBE_SPRING_PCM_ID = 2;

/**
 * Drive Subsystem
 */

// CAN IDs
static const int LEFT_DRIVE_A_CAN_ID = 16;
static const int LEFT_DRIVE_B_VICTOR_ID = 15;
static const int LEFT_DRIVE_C_VICTOR_ID = 14;
static const int RIGHT_DRIVE_A_CAN_ID = 1;
static const int RIGHT_DRIVE_B_VICTOR_ID = 2;
static const int RIGHT_DRIVE_C_VICTOR_ID = 3;
static const int INTAKE_CAN_ID = 4;

// Constants
static constexpr double DRIVE_WIDTH = 25.205;
static constexpr double DRIVE_WHEEL_DIAMETER = 5.0;
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

// CAN IDs
static const int PCM_CAN_ID = 17;

// Digital Inputs
static const int PRESSURE_DIN_ID = 1;

// Relays
static const int COMPRESSOR_RELAY = 0;

/**
 * Joysticks
 */

// Port IDs
static const int DRIVER_JOYSTICK_PORT = 0;
static const int OPERATOR_JOYSTICK_PORT = 1;
}
