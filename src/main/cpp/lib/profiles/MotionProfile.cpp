/**
 *
 * @author kyledematias
 */

#include "lib/profiles/MotionProfile.h"
#include "lib/util/WrapDash.h"
#include <cmath>
namespace frc973 {

namespace Profiler {
NewWaypoint TrapezoidProfileUnsafe(double time, double distance, double angle,
                                   double max_velocity, double acceleration,
                                   double start_velocity, double end_velocity) {
    double t0 = 0.0, t1, t2, t3;
    double d1 = 0.0, d2, d3;
    double velocity_now = 0;
    double dist_now = 0;

    if (start_velocity == end_velocity && max_velocity == start_velocity) {
        // trapezoid profile also handles a rectangular profile
        t1 = 0.0;
        t2 = (distance / max_velocity);
        t3 = t2;
        d1 = distance;
        d2 = distance;
        d3 = distance;
    }
    else {
        t0 = 0.0;
        t1 = (fabs(max_velocity) - fabs(start_velocity)) / acceleration;
        d1 = ((fabs(max_velocity) + fabs(start_velocity)) / 2.0) * t1;
        d3 = (Util::square(max_velocity) - Util::square(end_velocity)) /
             (2.0 * acceleration);
        d2 = fabs(distance) - d1 - d3;
        t2 = fabs(d2 / max_velocity) + t1;
        t3 = (fabs(end_velocity) - fabs(max_velocity)) / -acceleration + t2;
    }

    if (time < t1 && time >= t0) {
        // ramping
        velocity_now = fabs(start_velocity) + acceleration * time;
        dist_now = (0.5 * velocity_now * time);
    }
    else if (time >= t1 && time <= t2) {
        // coasting
        velocity_now = fabs(max_velocity);
        dist_now = d1 + (max_velocity * (time - t1));
    }
    else if (time > t2 && time <= t3) {
        // halting
        velocity_now = fabs(max_velocity) - acceleration * (time - t2);
        dist_now = d1 + (max_velocity * (t2 - t1)) +
                   ((Util::square(velocity_now) - Util::square(max_velocity)) /
                    (2.0 * -acceleration));
    }
    else if (time < t0) {
        // pre profile happenings
        velocity_now = 0.0;
        dist_now = 0.0;
    }
    else if (time > t3) {
        // post profile happenings
        velocity_now = end_velocity;
        dist_now = distance;
    }

    DBStringPrintf(DB_LINE2, "dn%0.1lf vn%0.1lf", dist_now, velocity_now);

    if (time < t0) {
        // pre mortem
        return NewWaypoint(time, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    else if (t0 <= time && time < t1) {
        // ramping up
        double doneness =
            dist_now / distance * Util::signum(distance) * Util::signum(angle);
        double angle_now = angle * doneness;
        double angle_vel_now = velocity_now * angle / distance *
                               Util::signum(distance) * Util::signum(angle);

        return NewWaypoint(time, velocity_now * Util::signum(distance),
                           dist_now * Util::signum(distance),
                           angle_vel_now * Util::signum(angle),
                           angle_now * Util::signum(angle), false, false);
    }
    else if (t1 <= time && time < t2) {
        // coasting
        double doneness =
            dist_now / distance * Util::signum(distance) * Util::signum(angle);
        double angle_now = angle * doneness;
        double angle_vel_now = velocity_now * angle / distance *
                               Util::signum(distance) * Util::signum(angle);

        return NewWaypoint(time, velocity_now * Util::signum(distance),
                           dist_now * Util::signum(distance),
                           angle_vel_now * Util::signum(angle),
                           angle_now * Util::signum(angle), false, false);
    }
    else if (t2 <= time && time < t3) {
        // halting
        double doneness = fabs(dist_now / distance);
        double angle_now = fabs(angle * doneness);
        double angle_vel_now = fabs(velocity_now * angle / distance);

        return NewWaypoint(time, velocity_now * Util::signum(distance),
                           dist_now * Util::signum(distance),
                           angle_vel_now * Util::signum(angle),
                           angle_now * Util::signum(angle), false, false);
    }
    else {
        // t3 <= time
        // post mortem
        if (end_velocity == 0.0) {
            return NewWaypoint(time, 0.0, distance, 0.0, angle, true, false);
        }
        else {
            double angle_vel_now = velocity_now * angle / distance *
                                   Util::signum(distance) * Util::signum(angle);

            return NewWaypoint(time, velocity_now * Util::signum(distance),
                               distance, angle_vel_now * Util::signum(angle),
                               angle, true, false);
        }
    }
}
NewWaypoint TriProfileUnsafe(double time, double distance, double angle,
                             double max_velocity, double acceleration,
                             double start_velocity, double end_velocity) {
    double cap_velocity =
        sqrt(fabs(acceleration * distance) +
             (Util::square(start_velocity) + Util::square(end_velocity)) / 2.0);
    double t0 = 0.0;
    double t_half = (fabs(cap_velocity) - fabs(start_velocity)) / acceleration;
    double t1 =
        fabs((fabs(end_velocity) - fabs(cap_velocity))) / acceleration + t_half;

    double velocity_now = 0;
    double dist_now = 0;

    if (time == t_half) {
        velocity_now = acceleration * t_half + fabs(start_velocity);
        dist_now = (velocity_now * t_half);
    }
    else if (time < t_half && time >= t0) {
        velocity_now = acceleration * time + fabs(start_velocity);
        dist_now = (0.5 * velocity_now * time);
    }
    else if (time > t_half && time <= t1) {
        velocity_now = fabs(cap_velocity) - acceleration * (time - t_half);
        dist_now = (cap_velocity * t_half) / 2.0 +
                   (cap_velocity + velocity_now) / 2.0 * (time - t_half);
    }
    else {
        velocity_now = 0.0;
        dist_now = 0.0;
    }

    DBStringPrintf(DB_LINE2, "vc %0.1lf dn%0.1lf vn%0.1lf", cap_velocity,
                   dist_now, velocity_now);

    if (time < t0) {
        // pre mortem
        return NewWaypoint(time, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    else if (t0 <= time && time < t_half) {
        // ramping up
        double doneness =
            dist_now / distance * Util::signum(distance) * Util::signum(angle);
        double angle_now = angle * doneness;
        double angle_vel_now = velocity_now * angle / distance *
                               Util::signum(distance) * Util::signum(angle);

        return NewWaypoint(time, velocity_now * Util::signum(distance),
                           dist_now * Util::signum(distance),
                           angle_vel_now * Util::signum(angle),
                           angle_now * Util::signum(angle), false, false);
    }
    else if (time == t_half) {
        double doneness =
            dist_now / distance * Util::signum(distance) * Util::signum(angle);
        double angle_now = angle * doneness;
        double angle_vel_now = velocity_now * angle / distance *
                               Util::signum(distance) * Util::signum(angle);

        return NewWaypoint(time, velocity_now * Util::signum(distance),
                           dist_now * Util::signum(distance),
                           angle_vel_now * Util::signum(angle),
                           angle_now * Util::signum(angle), false, false);
    }
    else if (t_half <= time && time < t1) {
        // halting
        double doneness = fabs(dist_now / distance);
        double angle_now = fabs(angle * doneness);
        double angle_vel_now = fabs(velocity_now * angle / distance);

        return NewWaypoint(time, velocity_now * Util::signum(distance),
                           dist_now * Util::signum(distance),
                           angle_vel_now * Util::signum(angle),
                           angle_now * Util::signum(angle), false, false);
    }
    else {
        // post mortem
        if (end_velocity == 0.0) {
            return NewWaypoint(time, 0.0, distance, 0.0, angle, true, false);
        }
        else {
            double angle_vel_now = velocity_now * angle / distance *
                                   Util::signum(distance) * Util::signum(angle);

            return NewWaypoint(time, velocity_now * Util::signum(distance),
                               distance, angle_vel_now * Util::signum(angle),
                               angle, true, false);
        }
    }
}
}
}
