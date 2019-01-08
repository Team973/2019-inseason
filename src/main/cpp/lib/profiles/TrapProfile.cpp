#include "lib/profiles/TrapProfile.h"
#include <math.h>

namespace frc973 {

namespace Profiler {

Waypoint TrapProfileUnsafe(double time, double distance, double angle,
                           double max_velocity, double max_acceleration,
                           bool start_halt, bool end_halt) {
    double dist_ramp = 0.5 * max_velocity * max_velocity / max_acceleration *
                       Util::signum(distance);

    double t_0 = 0.0, t_1, t_2, t_3;

    if (start_halt) {
        // start with zero velocity
        if (end_halt) {
            // end with zero velocity
            if (fabs(2 * dist_ramp) < fabs(distance)) {
                // the move is long enough to get up to speed
                t_1 = max_velocity / max_acceleration;
                double dist_during_const_vel = distance - (2 * dist_ramp);
                t_2 = fabs(dist_during_const_vel) / max_velocity + t_1;
                t_3 = t_2 + t_1;
            }
            else {
                // the move is not long enough so do a triangle
                t_1 = sqrt(fabs(distance / max_acceleration));
                t_2 = t_1;
                t_3 = 2.0 * t_1;
            }
        }
        else {
            // don't slow down at end of move
            if (fabs(dist_ramp) < fabs(distance)) {
                // we have enough time to get up to speed
                t_1 = max_velocity / max_acceleration;
                double dist_during_const_vel = distance - dist_ramp;
                t_2 = fabs(dist_during_const_vel) / max_velocity + t_1;
                t_3 = t_2;
            }
            else {
                // we don't have enough time to get to speed so just left tri
                t_1 = sqrt(fabs(2.0 * distance / max_acceleration));
                t_2 = t_1;
                t_3 = t_1;
            }
        }
    }
    else {
        // start with max velocity
        if (end_halt) {
            // end with zero velocity
            // this is the case were an impossible profile could occur but
            // we already checked for that statically
            if (fabs(dist_ramp) > fabs(distance)) {
                return Waypoint(time, 0.0, 0.0, 0.0, 0.0, false, true);
            }

            t_1 = 0.0;
            double dist_during_const_vel = distance - dist_ramp;
            t_2 = fabs(dist_during_const_vel) / max_velocity;
            t_3 = t_2 + max_velocity / max_acceleration;
        }
        else {
            // we end with max velocity
            // i.e. maintain max velocity the entire time
            t_1 = 0.0;
            t_2 = fabs(distance / max_velocity);
            t_3 = t_2;
        }
    }

    /*
    printf("t_0=%lf t_1=%lf t_2=%lf t_3=%lf\n",
           t_0, t_1, t_2, t_3);
    */

    if (time < t_0) {
        // pre mortem
        return Waypoint(time, 0.0, 0.0, 0.0, 0.0, false, false);
    }
    else if (t_0 <= time && time < t_1) {
        // ramping up
        double velocity_now = max_acceleration * time;
        double dist_now = 0.5 * time * velocity_now;

        double doneness =
            dist_now / distance * Util::signum(distance) * Util::signum(angle);
        double angle_now = angle * doneness;
        double angle_vel_now = velocity_now * angle / distance *
                               Util::signum(distance) * Util::signum(angle);

        return Waypoint(time, velocity_now * Util::signum(distance),
                        dist_now * Util::signum(distance),
                        angle_vel_now * Util::signum(angle),
                        angle_now * Util::signum(angle), false, false);
    }
    else if (t_1 <= time && time < t_2) {
        // coasting
        double velocity_now = max_velocity;
        double time_in_phase = time - t_1;
        double dist_in_phase = time_in_phase * velocity_now;
        double dist_now;
        if (start_halt) {
            dist_now = fabs(dist_ramp) + dist_in_phase;
        }
        else {
            dist_now = dist_in_phase;
        }

        double doneness =
            dist_now / distance * Util::signum(distance) * Util::signum(angle);
        double angle_now = angle * doneness;
        double angle_vel_now = velocity_now * angle / distance *
                               Util::signum(distance) * Util::signum(angle);

        return Waypoint(time, velocity_now * Util::signum(distance),
                        dist_now * Util::signum(distance),
                        angle_vel_now * Util::signum(angle),
                        angle_now * Util::signum(angle), false, false);
    }
    else if (t_2 <= time && time < t_3) {
        // halting
        double time_till_end = t_3 - time;
        double velocity_now = max_acceleration * time_till_end;
        double dist_till_end = 0.5 * time_till_end * velocity_now;
        double dist_now = fabs(distance) - dist_till_end;

        double doneness = fabs(dist_now / distance);
        double angle_now = fabs(angle * doneness);
        double angle_vel_now = fabs(velocity_now * angle / distance);

        return Waypoint(time, velocity_now * Util::signum(distance),
                        dist_now * Util::signum(distance),
                        angle_vel_now * Util::signum(angle),
                        angle_now * Util::signum(angle), false, false);
    }
    else {
        // t_3 <= time
        // post mortem
        if (end_halt) {
            return Waypoint(time, 0.0, distance, 0.0, angle, true, false);
        }
        else {
            double velocity_now;
            if (start_halt) {
                velocity_now = Util::min(max_velocity, max_acceleration * t_3);
            }
            else {
                velocity_now = max_velocity;
            }

            double angle_vel_now = velocity_now * angle / distance *
                                   Util::signum(distance) * Util::signum(angle);

            return Waypoint(time, velocity_now * Util::signum(distance),
                            distance, angle_vel_now * Util::signum(angle),
                            angle, true, false);
        }
    }
}
}
}
