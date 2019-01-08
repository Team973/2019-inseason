#pragma once

#include "lib/trajectories/structs.h"
#include "stdio.h"
#include <cmath>

namespace trajectories {

/**
 * Get the left drive velocity.
 * @param trajectory The trajectory.
 * @param time The time into the trajectory.
 * @return The left drive velocity.
 */
double GetLeftDriveVelocity(TrajectoryDescription *trajectory, double time);

/**
 * Get the right drive velocity.
 * @param trajectory The trajectory.
 * @param time The time into the trajectory.
 * @return The right drive velocity.
 */
double GetRightDriveVelocity(TrajectoryDescription *trajectory, double time);

/**
 * Get the left drive distance.
 * @param trajectory The trajectory.
 * @param time The time into the trajectory.
 * @return The left drive distance.
 */
double GetLeftDist(TrajectoryDescription *trajectory, double time);

/**
 * Get the right drive distance.
 * @param trajectory The trajectory.
 * @param time The time into the trajectory.
 * @return The right drive distance.
 */
double GetRightDist(TrajectoryDescription *trajectory, double time);

/**
 * Get the left drive acceleration.
 * @param trajectory The trajectory.
 * @param time The time into the trajectory.
 * @return The left drive acceleration.
 */
double GetLeftAcceleration(TrajectoryDescription *trajectory, double time);

/**
 * Get the right drive acceleration.
 * @param trajectory The trajectory.
 * @param time The time into the trajectory.
 * @return The right drive acceleration.
 */
double GetRightAcceleration(TrajectoryDescription *trajectory, double time);

/**
 * Get the heading degrees.
 * @param trajectory The trajectory.
 * @param time The time into the trajectory.
 * @return The heading degrees.
 */
double GetHeadingDegrees(TrajectoryDescription *trajectory, double time);

/**
 * Get the length of the trajectory.
 * @param trajectory The trajectory.
 * @return The length of the trajectory.
 */
int GetLength(TrajectoryDescription *trajectory);

/**
 * Get the percent complete of the trajectory.
 * @param trajectory The trajectory.
 * @param time The time into the trajectory.
 * @return The percent complete.
 */
double GetPercentComplete(TrajectoryDescription *trajectory, double time);

/**
 * Get the angular rate degrees.
 * @param trajectory The trajectory.
 * @param time The time into the trajectory.
 * @return The angular rate degrees.
 */
double GetAngularRateDegrees(TrajectoryDescription *trajectory, double time);

/**
 * Get the angular acceleration.
 * @param trajectory The trajectory.
 * @param time The time into the trajectory.
 * @return The angular acceleration.
 */
double GetAngularAcceleration(TrajectoryDescription *trajectory, double time);
}
