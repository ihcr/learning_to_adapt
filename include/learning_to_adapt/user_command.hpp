#pragma once

#include <Eigen/Dense>

namespace bio_gait {

struct UserCommand
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Vector3d Vector3d;

    UserCommand() {
        gait_type = 0;
        gait_override = 0;
        gait_period_time = 0.;
        gait_switching_phase = 0.;
        gait_step_height = 0.;

        des_base_lin_vel.setZero();
        des_base_ang_vel.setZero();
    }

    // Gait Scheduler
    int gait_type;
    int gait_override;
    double gait_period_time;
    double gait_switching_phase;
    double gait_step_height;

    // Planner
    Vector3d des_base_lin_vel;
    Vector3d des_base_ang_vel;
};

}  // end quad_gait namespace
