#pragma once

#include "lib/util/Util.h"
#include "lib/util/WrapDash.h"

namespace frc973 {

/**
 * Holds motion profilers.
 */
namespace Profiler {

/**
 * NewWaypoint represents an intermittent setpoint in a motion profile.
 */
struct NewWaypoint {
    double time; /**< The time. */

    double linear_dist; /**< The target linear distance. */
    double linear_vel;  /**< The target linear velocity. */

    double angular_dist; /**< The target angular distance. */
    double angular_vel;  /**< The target angular velocity. */

    bool done;  /**< Whether it's done. */
    bool error; /**< Whether it errored. */

    /**
     * Construct a NewWaypoint.
     * @param time_ The time.
     * @param linear_vel_ The target linear velocity.
     * @param linear_dist_ The target linear distance.
     * @param angular_vel_ The target angular velocity.
     * @param angular_dist_ The target angular distance.
     * @param done_ Whether it's done.
     * @param error_ Whether it errored.
     */
    NewWaypoint(double time_, double linear_vel_, double linear_dist_,
                double angular_vel_, double angular_dist_, bool done_,
                bool error_)
            : time(time_)
            , linear_dist(linear_dist_)
            , linear_vel(linear_vel_)
            , angular_dist(angular_dist_)
            , angular_vel(angular_vel_)
            , done(done_)
            , error(error_) {
    }

    /**
     * Constuct a new zeroed waypoint.
     */
    NewWaypoint()
            : time(0.0)
            , linear_dist(0.0)
            , linear_vel(0.0)
            , angular_dist(0.0)
            , angular_vel(0.0)
            , done(false)
            , error(false) {
    }
};

/**
 * TrapezoidProfileUnsafe does the calculation at runtime like one would
 * expect and is a normal function. Do not call this function directly, it is
 * dangerous. Instead, call TrapProfile.
 * @param time The time.
 * @param distance The distance.
 * @param angle The angle.
 * @param max_velocity The maximum velocity.
 * @param acceleration The acceleration.
 * @param start_velocity The start velocity.
 * @param end_velocity The end velocity.
 * @return The NewWaypoint.
 */
NewWaypoint TrapezoidProfileUnsafe(double time, double distance, double angle,
                                   double max_velocity, double acceleration,
                                   double start_velocity, double end_velocity);

/**
 * TriProfileUnsafe.
 * @param time The time.
 * @param distance The distance.
 * @param angle The angle.
 * @param max_velocity The maximum velocity.
 * @param acceleration The acceleration.
 * @param start_velocity The start velocity.
 * @param end_velocity The end velocity.
 * @return The NewWaypoint.
 */
NewWaypoint TriProfileUnsafe(double time, double distance, double angle,
                             double max_velocity, double acceleration,
                             double start_velocity, double end_velocity);
}
}
