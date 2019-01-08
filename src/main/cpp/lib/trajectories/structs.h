#pragma once

namespace trajectories {

/**
 * The struct that creates segments of trajectories.
 */
struct Segment {
    double dt;            /**< Amount of dt. */
    double x;             /**< X position. */
    double y;             /**< Y position. */
    double position;      /**< Position to reach. */
    double velocity;      /**< Amount of velocity. */
    double acceleration;  /**< Amount of acceleration. */
    double jerk;          /**< Amount of jerk. */
    double heading;       /**< Heading to reach. */
    double angular_rate;  /**< Amount of angular rate. */
    double angular_accel; /**< Amount of angular acceleration. */
};

/**
 * The struct that describes trajectories.
 */
struct TrajectoryDescription {
    double timestep;           /**< The timestep. */
    double max_vel;            /**< The maximum velocity. */
    double max_accel;          /**< The maximum acceleration. */
    double max_jerk;           /**< The maximum jerk. */
    double wheelbase_width;    /**< The maximum velocity. */
    int length;                /**< The length. */
    Segment *left_trajectory;  /**< The left trajectory. */
    Segment *right_trajectory; /**< The right trajectory. */
};
}
