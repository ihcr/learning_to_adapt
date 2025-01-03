#pragma once

#include <Eigen/Dense>

#include "learning_to_adapt/gait_type.hpp"

namespace bio_gait {

/**
 * @brief Timing data for a gait.
 */
struct GaitData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Vector4i Vector4i;
    typedef Eigen::Vector4d Vector4d;
    typedef Eigen::Vector2d Vector2d;

    GaitData();

    void setZero();

    // Gait name
    std::string gait_name;

    // The current GaitType
    GaitType current_gait;
    // Next GaitType to transition into
    GaitType next_gait;

    // Gait descriptors
    double period_time_nominal;      // overall period time to scale
    double initial_phase;            // initial phase to offset
    double switching_phase_nominal;  // nominal phase to switch contacts
    int overrideable;

    // Enable flag for each leg
    Vector4i gait_enabled;  // enable gait controlled legs

    // Time based descriptors
    Vector4d period_time;            // overall leg scaled gait period time
    Vector4d time_stance;            // total stance time
    Vector4d time_swing;             // total swing time
    Vector4d time_stance_remaining;  // stance time remaining
    Vector4d time_swing_remaining;   // swing time remaining

    // Phase based descriptors
    Vector4d switching_phase;  // phase to switch to swing
    Vector4d phase_variable;   // overall gait phase for each leg
    Vector4d phase_offset;     // nominal gait phase offsets
    Vector4d phase_scale;      // phase scale relative to variable
    Vector4d phase_stance;     // stance subphase
    Vector4d phase_swing;      // swing subphase

    // Scheduled contact states
    Vector4i contact_state_scheduled;  // contact state of the leg
    Vector4i contact_state_prev;       // previous contact state of the leg
    Vector4i touchdown_scheduled;      // scheduled touchdown event flag
    Vector4i liftoff_scheduled;        // scheduled touchdown event flag
    
    // Transition descriptors
    Vector2d vel_bounds;    // upper and lower velocity bounds for transition (m/s)
    Vector2d fr_bounds;     // upper and lower Froude number bounds for transition
    double L;               // number of frames of the gait per cycle
    double B;               // number of time frames of transition
    double Cmax;
    double C;
    double dt;              // change of phase of each frame of the transition
    double trans_flag;
    double FrStab;
    
    Eigen::Vector3d froudeCmd_;

    // Simple foot trajectories
    Vector4d des_foot_height;
    
    // foot traj stuff
    Vector4d ref_foot_x;
    Vector4d ref_foot_y;
    Vector4d ref_foot_z;

};

}  // end bio_gait namespace
