#pragma once

#include "frc/WPILib.h"

using namespace frc;

namespace frc973 {

/**
 * Used internally to represent the state of the robot.
 */
enum RobotMode
{
    MODE_DISABLED, /**< Disabled mode. */
    MODE_AUTO,     /**< Autonomous mode. */
    MODE_TELEOP,   /**< Teleop mode. */
    MODE_TEST      /**< Test mode. */
};

/**
 * GetRobotMode queries the provided RobotState and returns the mode
 * the robot is running in.
 * @param stateProvider The RobotState to query to get running mode.
 * @return The current robot mode in RobotMode format.
 */
RobotMode GetRobotMode(RobotState &stateProvider);

/**
 * Constants
 */
namespace Constants {
constexpr double PI = 3.141592653589793;                 /**< Pi. */
constexpr double FEET_PER_METER = 3.280839895;           /**< ft/m. */
constexpr double METERS_PER_FOOT = 1.0 / FEET_PER_METER; /**< m/ft. */
constexpr double GRAVITY_CONSTANT =
    9.80665; /**< Gravity constant meter/sq(sec). */
constexpr double GRAVITY_CONSTANT_INCHES =
    GRAVITY_CONSTANT * FEET_PER_METER *
    12.0; /**< Gravity constant in/sq(sec). */

constexpr double USEC_PER_MSEC = 1000.0; /**< Microseconds/millisecond. */
constexpr double MSEC_PER_USEC =
    1.0 / USEC_PER_MSEC; /**< Milliseconds/microseconds. */

constexpr double MSEC_PER_SEC = 1000.0; /**< Milliseconds/seconds. */
constexpr double SEC_PER_MSEC =
    1.0 / MSEC_PER_SEC; /**< Seconds/milliseconds. */

constexpr double USEC_PER_SEC =
    USEC_PER_MSEC * MSEC_PER_SEC; /**< Microseconds/second. */
constexpr double SEC_PER_USEC =
    1.0 / USEC_PER_SEC; /**< Microseconds/milliseconds. */

constexpr double MIN_PER_SEC = 1.0 / 60.0; /**< Minutes/seconds. */
constexpr double SEC_PER_MIN = 60.0;       /**< Seconds/minute. */

constexpr double RAD_PER_DEG = 2 * PI / 360.0;    /**< Radians/degrees. */
constexpr double DEG_PER_RAD = 1.0 / RAD_PER_DEG; /**< Degrees/Radians. */
}

/**
 * Macros
 */
#define ARRAYSIZE(a) (sizeof(a) / sizeof((a)[0]))

/**
 * Finds the magnitude by sqrt(x^2 + y^2).
 * @return The magnitude.
 */
inline double magnitude(double x, double y) {
    return sqrt(pow(x, 2.0) + pow(y, 2.0));
}

/**
 * Get the current time in microseconds.
 * @return The current time.
 */
inline uint64_t GetUsecTime() {
    return RobotController::GetFPGATime();
}

/**
 * Get the current time in milliseconds.
 * @return The current time.
 */
inline uint32_t GetMsecTime() {
    return GetUsecTime() * Constants::MSEC_PER_USEC;
}

/**
 * Get the current time in seconds.
 * @return The current time.
 */
inline double GetSecTime() {
    return GetUsecTime() * Constants::SEC_PER_USEC;
}

namespace Util {

/**
 * Return |val| coerced to be above |low| and below |high| inclusive.
 * @param val
 * @param low
 * @param high
 */
inline double bound(double val, double low, double high) {
    if (val < low) {
        return low;
    }
    else if (val > high) {
        return high;
    }
    else {
        return val;
    }
}

/**
 * Return the lesser of the two given numbers.
 * @param a The first number.
 * @param b The second number.
 * @return The lesser number.
 */
inline double min(double a, double b) {
    if (a < b) {
        return a;
    }
    else {
        return b;
    }
}

/**
 * Return the greater of the two given numbers.
 * @param a The first number.
 * @param b The second number.
 * @return The greater number.
 */
inline double max(double a, double b) {
    if (a > b) {
        return a;
    }
    else {
        return b;
    }
}

/**
 * Return the limit of the two given numbers.
 * @param x
 * @param maxMagnitude
 * @return The limit of the numbers.
 */
inline double limit(double x, double maxMagnitude) {
    return fmin(maxMagnitude, fmax(-maxMagnitude, x));
}

/**
 * Calculate the angle error.
 * @param x
 * @param y
 * @return
 */
inline double CalcAngleError(double x, double y) {
    double ret = std::fmod(x - y + 180.0, 360) - 180;
    if (ret < -180) {
        return ret + 360.0;
    }
    else {
        return ret;
    }
}

/**
 * Return 0 if |n| is within +/- |threshold|, otherwise return |n|. Useful for
 * joysticks that aren't quite centered at zero.
 * @param n
 * @param threshold
 * @return 0 if |n| is within +/- |threshold|, otherwise return |n|.
 */
inline double deadband(double n, double threshold) {
    if (fabs(n) > threshold) {
        return n;
    }
    else {
        return 0.0;
    }
}

/**
 * Return the number if fabs(n) > threshold, otherwise return the threshold with
 * the sign of the number. If the number is 0.0, return the threshold.
 * @param n The number.
 * @param threshold The threshold.
 * @return
 */
inline double antideadband(double n, double threshold) {
    if (fabs(n) < threshold) {
        if (n < 0.0) {
            return -threshold;
        }
        else {
            return threshold;
        }
    }
    else {
        return n;
    }
}

/**
 * Increase the given number by the increase amount but respeting the sign of
 * the number. If the number is positive, increase it, if the number is
 * negative, decrease it.
 * @param x The number.
 * @param increase The amount to increase.
 * @return The signed increase.
 */
inline double signedIncrease(double x, double increase) {
    if (x >= 0.0) {
        return x + increase;
    }
    else {
        return x - increase;
    }
}

/**
 * Square the given number, but keep the sign the same.
 * @param n The number.
 * @return The square of the number with the same sign.
 */
inline double signSquare(double n) {
    if (n < 0.0) {
        return -1.0 * n * n;
    }
    else {
        return n * n;
    }
}

/**
 * Square the given number.
 * @param n The number.
 * @return The square of the number.
 */
inline double square(double n) {
    return n * n;
}

/**
 * Returns true if a and b are close (within epsilon) to each other.
 * @param a The first number.
 * @param b The second number.
 * @param epsilon The range.
 * @return Whether the number is within the epsilon range.
 */
inline bool close(double a, double b, double epsilon = 0.00001) {
    return fabs(a - b) < epsilon;
}

/**
 * Transforms a value (n) in the range between (sl) and (sh) to the range
 * between (tl) and (th).
 */
inline double normalize(double n, double sl, double sh, double tl, double th) {
    return (n - sl) * (th - tl) / (sh - sl) + tl;
}

/**
 * Return a unit scalar with the same sign as the argument
 */
constexpr inline double signum(double x) {
    return (x < 0) ? -1 : (x > 0) ? 1 : 0;
}

/**
 * A point on a 2d plane.
 */
struct Point {
    double x; /**< The x coordinate.*/
    double y; /**< The y coordinate.*/

    /**
     * Construct a point.
     * @param _X The x coordinate.
     * @param _Y The y coordinate.
     */
    Point(double _X, double _Y) : x(_X), y(_Y) {
    }
};

/**
 * Given two points on a line, return the y value on this point at the given x.
 * @param a The first point.
 * @param b The second point.
 * @param x The x coordinate.
 * @return The y value on the line.
 */
inline double interpolate(Point a, Point b, double x) {
    double slope = (b.y - a.y) / (b.x - a.x);
    double intercept = a.y - (slope * a.x);
    return slope * x + intercept;
}
}  // Util

}  // frc973
