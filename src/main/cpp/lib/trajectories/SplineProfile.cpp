#include "lib/trajectories/SplineProfile.h"
#include "lib/trajectories/structs.h"
#include "lib/util/Util.h"

using namespace frc973;

namespace trajectories {
double GetLeftDriveVelocity(TrajectoryDescription *trajectory, double time) {
    int index = (int)(time / trajectory->left_trajectory->dt);
    if (index >= GetLength(trajectory)) {
        return trajectory->left_trajectory[GetLength(trajectory) - 1].velocity;
    }
    return trajectory->left_trajectory[index].velocity;
}

double GetRightDriveVelocity(TrajectoryDescription *trajectory, double time) {
    int index = (int)(time / trajectory->right_trajectory->dt);
    if (index >= GetLength(trajectory)) {
        return trajectory->right_trajectory[GetLength(trajectory) - 1].velocity;
    }
    return trajectory->right_trajectory[index].velocity;
}

double GetLeftDist(TrajectoryDescription *trajectory, double time) {
    int index = (int)(time / trajectory->left_trajectory->dt);
    if (index >= GetLength(trajectory)) {
        return trajectory->left_trajectory[GetLength(trajectory) - 1].position;
    }
    return trajectory->left_trajectory[index].position;
}

double GetRightDist(TrajectoryDescription *trajectory, double time) {
    int index = (int)(time / trajectory->right_trajectory->dt);
    if (index >= GetLength(trajectory)) {
        return trajectory->right_trajectory[GetLength(trajectory) - 1].position;
    }
    return trajectory->right_trajectory[index].position;
}

double GetLeftAcceleration(TrajectoryDescription *trajectory, double time) {
    int index = (int)(time / trajectory->left_trajectory->dt);
    if (index >= GetLength(trajectory)) {
        return trajectory->left_trajectory[GetLength(trajectory) - 1]
            .acceleration;
    }
    return trajectory->left_trajectory[index].acceleration;
}

double GetRightAcceleration(TrajectoryDescription *trajectory, double time) {
    int index = (int)(time / trajectory->right_trajectory->dt);
    if (index >= GetLength(trajectory)) {
        return trajectory->right_trajectory[GetLength(trajectory) - 1]
            .acceleration;
    }
    return trajectory->right_trajectory[index].acceleration;
}

double GetHeadingDegrees(TrajectoryDescription *trajectory, double time) {
    int index = (int)(time / trajectory->left_trajectory->dt);
    if (index >= GetLength(trajectory)) {
        return trajectory->left_trajectory[GetLength(trajectory) - 1].heading *
               180.0 / Constants::PI;
    }
    return trajectory->left_trajectory[index].heading * 180.0 / Constants::PI;
}

int GetLength(TrajectoryDescription *trajectory) {
    return trajectory->length;
}

double GetPercentComplete(TrajectoryDescription *trajectory, double time) {
    int index = (int)(time / trajectory->left_trajectory->dt);
    return (float)(index) / (float)(GetLength(trajectory));
}

double GetAngularRateDegrees(TrajectoryDescription *trajectory, double time) {
    int index = (int)(time / trajectory->left_trajectory->dt);
    if (index >= GetLength(trajectory)) {
        return trajectory->right_trajectory[GetLength(trajectory) - 1]
                   .angular_rate *
               Constants::DEG_PER_RAD;
    }
    return trajectory->left_trajectory[index].angular_rate *
           Constants::DEG_PER_RAD;
}

double GetAngularAcceleration(TrajectoryDescription *trajectory, double time) {
    int index = (int)(time / trajectory->left_trajectory->dt);
    if (index >= GetLength(trajectory)) {
        return trajectory->right_trajectory[GetLength(trajectory) - 1]
                   .angular_accel *
               Constants::DEG_PER_RAD;
    }
    return trajectory->left_trajectory[index].angular_accel *
           Constants::DEG_PER_RAD;
}
}
