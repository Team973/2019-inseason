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
 * @return The current RobotMode.
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

/**
 * Holds utility functions.
 */
namespace Util {

/**
 * Gets |x| coerced to be above |low| and below |high| inclusive.
 * @param x The value.
 * @param low The low threshold.
 * @param high the high threshold.
 */
inline double bound(double x, double low, double high) {
    if (x < low) {
        return low;
    }
    else if (x > high) {
        return high;
    }
    else {
        return x;
    }
}

/**
 * Gets the lesser of the two given values.
 * @param x The first value.
 * @param y The second value.
 * @return The lesser value.
 */
inline double min(double x, double y) {
    if (x < y) {
        return x;
    }
    else {
        return y;
    }
}

/**
 * Gets the greater of the two given values.
 * @param x The first value.
 * @param y The second value.
 * @return The greater value.
 */
inline double max(double x, double y) {
    if (x > y) {
        return x;
    }
    else {
        return y;
    }
}

/**
 * Gets the limit of the two given values.
 * @param x The value.
 * @param maxMagnitude
 * @return The limit of the values.
 */
inline double limit(double x, double maxMagnitude) {
    return fmin(maxMagnitude, fmax(-maxMagnitude, x));
}

/**
 * Calculate the angle error.
 * @param x The target angle.
 * @param y The actual angle.
 * @return The angle error.
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
 * Gets 0 if |x| is within +/- |threshold|, otherwise return |x|. Useful for
 * joysticks that aren't quite centered at zero.
 * @param x
 * @param threshold
 * @return 0 if |x| is within +/- |threshold|, otherwise return |x|.
 */
inline double deadband(double x, double threshold) {
    if (fabs(x) > threshold) {
        return x;
    }
    else {
        return 0.0;
    }
}

/**
 * Gets the value if fabs(x) > threshold, otherwise return the threshold with
 * the sign of the value. If the value is 0.0, return the threshold.
 * @param x The value.
 * @param threshold The threshold.
 * @return
 */
inline double antideadband(double x, double threshold) {
    if (fabs(x) < threshold) {
        if (x < 0.0) {
            return -threshold;
        }
        else {
            return threshold;
        }
    }
    else {
        return x;
    }
}

/**
 * Increase the given value by the increase amount but respeting the sign of
 * the value. If the value is positive, increase it, if the value is
 * negative, decrease it.
 * @param x The value.
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
 * Square the given value, but keep the sign the same.
 * @param x The value.
 * @return The square of the value with the same sign.
 */
inline double signSquare(double x) {
    if (x < 0.0) {
        return -1.0 * x * x;
    }
    else {
        return x * x;
    }
}

/**
 * Square the given value.
 * @param x The value.
 * @return The square of the value.
 */
inline double square(double x) {
    return x * x;
}

/**
 * Gets true if a and b are close (within epsilon) to each other.
 * @param x The first value.
 * @param y The second value.
 * @param epsilon The range.
 * @return Whether the value is within the epsilon range.
 */
inline bool close(double x, double y, double epsilon = 0.00001) {
    return fabs(x - y) < epsilon;
}

/**
 * Transforms a value (x) in the range between (sl) and (sh) to the range
 * between (tl) and (th).
 * @param x The value.
 * @param sl The low s threshold.
 * @param sh The high s threshold.
 * @param tl The low t threshold.
 * @param th The high t threshold.
 * @return The normalized value.
 */
inline double normalize(double x, double sl, double sh, double tl, double th) {
    return (x - sl) * (th - tl) / (sh - sl) + tl;
}

/**
 * Gets a unit scalar with the same sign as the argument.
 * @param x The value.
 * @return The unit scalar.
 */
constexpr inline double signum(double x) {
    return (x < 0) ? -1 : (x > 0) ? 1 : 0;
}

/**
 * A Point on a 2D plane.
 */
struct Point {
    double x; /**< The x coordinate. */
    double y; /**< The y coordinate. */

    /**
     * Construct a Point.
     * @param _X The x coordinate.
     * @param _Y The y coordinate.
     */
    Point(double _X, double _Y) : x(_X), y(_Y) {
    }
};

/**
 * Given two Point's on a line, return the y value on this Point at the given x.
 * @param a The first Point.
 * @param b The second Point.
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
