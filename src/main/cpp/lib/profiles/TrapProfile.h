#pragma once

#include "lib/util/Util.h"
#include "stdio.h"
#include <math.h>

namespace frc973 {

namespace Profiler {

/**
 * Waypoint represents an intermittent setpoint in a trap profile.
 */
struct Waypoint {
    double time; /**< The time.*/

    double linear_dist; /**< The target linear distance.*/
    double linear_vel;  /**< The target linear velocity.*/

    double angular_dist; /**< The target angular distance.*/
    double angular_vel;  /**< The target angular velocity.*/

    bool done;  /**< Whether it's done.*/
    bool error; /**< Whether it errored.*/

    /**
     * Construct a new waypoint.
     * @param time_ The time.
     * @param linear_vel_ The target linear velocity.
     * @param linear_dist_ The target linear distance.
     * @param angular_vel_ The target angular velocity.
     * @param angular_dist_ The target angular distance.
     * @param done_ Whether it's done.
     * @param error_ Whether it errored.
     */
    Waypoint(double time_, double linear_vel_, double linear_dist_,
             double angular_vel_, double angular_dist_, bool done_, bool error_)
            : time(time_)
            , linear_dist(linear_dist_)
            , linear_vel(linear_vel_)
            , angular_dist(angular_dist_)
            , angular_vel(angular_vel_)
            , done(done_)
            , error(error_) {
    }
};

/**
 * TrapProfileUnsafe does the calculation at runtime like one would expect and
 * is a normal function. Do not call this function directly, it is dangerous.
 * Instead, call TrapProfile.
 */
Waypoint TrapProfileUnsafe(double time, double distance, double angle,
                           double max_velocity, double max_acceleration,
                           bool start_halt, bool end_halt);

/**
 * Safely generates a trapazoidal motion profile. Checks at compile time for
 * profile safety.
 */
template <typename DISTANCE, typename ANGLE, typename MAX_VELOCITY,
          typename MAX_ACCELERATION, bool START_HALT, bool END_HALT>
Waypoint TrapProfile(double time) {
    constexpr double distance = DISTANCE::value;
    constexpr double angle = ANGLE::value;
    constexpr double max_velocity = MAX_VELOCITY::value;
    constexpr double max_acceleration = MAX_ACCELERATION::value;
    constexpr bool start_halt = START_HALT;
    constexpr bool end_halt = END_HALT;

    constexpr double dist_ramp = 0.5 * max_velocity * max_velocity /
                                 max_acceleration * Util::signum(distance);

    static_assert(
        !(!START_HALT && END_HALT) || dist_ramp < fabs(DISTANCE::value),
        "Profile is over-constrained");

    return TrapProfileUnsafe(time, distance, angle, max_velocity,
                             max_acceleration, start_halt, end_halt);
}
}
}
