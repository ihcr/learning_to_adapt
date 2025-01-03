#include "learning_to_adapt/gait_data.hpp"

namespace bio_gait {

GaitData::GaitData()
{
    this->setZero();
}

void GaitData::setZero()
{
    // Stop any gait transitions
    next_gait = current_gait;

    // General gait descriptors
    period_time_nominal = 0.0;
    initial_phase = 0.0;
    switching_phase_nominal = 0.0;
    overrideable = 0;

    // Enable flag for each leg
    gait_enabled.setZero();

    // Time based descriptors
    period_time.setZero();
    time_stance.setZero();
    time_swing.setZero();
    time_stance_remaining.setZero();
    time_swing_remaining.setZero();

    // Phase based descriptors
    switching_phase.setZero();
    phase_variable.setZero();
    phase_offset.setZero();
    phase_scale.setZero();
    phase_stance.setZero();
    phase_swing.setZero();

    // Scheduled contact states
    contact_state_scheduled.setZero();
    contact_state_prev.setZero();
    touchdown_scheduled.setZero();
    liftoff_scheduled.setZero();
    
    // Transition descriptors
    vel_bounds.setZero();
    
    trans_flag = 0;
    
    FrStab = 0;
    froudeCmd_.setZero();
    
    // Simple foot trajectories
    des_foot_height.setZero();
    
    // foot traj stuff
    ref_foot_x.setZero();
    ref_foot_y.setZero();
    ref_foot_z.setZero();
}

}  // end bio_gait namespace
