#pragma once

#include <Eigen/Dense>
#include <math.h>
#include <vector>

#include "learning_to_adapt/gait_type.hpp"
#include "learning_to_adapt/gait_data.hpp"
#include "learning_to_adapt/user_command.hpp"
#include "learning_to_adapt/foot_swing_traj.hpp"

namespace bio_gait {

constexpr int kNumLegs = 4;

struct FootRefs {
    Eigen::Vector3d position;
    Eigen::Vector3d positionLcl;
};


class GaitScheduler
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Vector3d Vector3d;
    typedef const Eigen::Ref<const Vector3d>& ConstRefVector3d;

    GaitScheduler(const double timestep, UserCommand& user_cmd, std::string trans_mode, const double height, const double foot_offset);

    void setupSwingFootTrajGen(const double clearance_ratio, const double foot_delta_x_limit, const double foot_delta_y_limit, const double raibert_kp, const Eigen::MatrixXd leg_endeff_rel_pos, const Eigen::MatrixXd hip_rel_pos, const Vector3d ref_base_pos, const Eigen::Vector4d ref_base_quat);

    const GaitData& gaitData() const;
    GaitData& gaitData();

    void setNominalHeight(const double h);

    /**
     * @brief Iteration step for scheduler logic
     * 
     * @param lin_vel Base linear velocity.
     * @param ang_vel Base angular velocity.
     */
    void step(ConstRefVector3d lin_vel, ConstRefVector3d ang_vel, const Eigen::Ref<const Eigen::VectorXd>& base_pose, const Eigen::Ref<const Eigen::VectorXd>& base_vel, const Eigen::Ref<const Eigen::VectorXd>& foot_heights);

    // Creates a new gait from predefined library
    void modifyGait();
    void createGait();
    void createNextGait(GaitType gaitId);
    void calcTransParams();
    void calcAuxiliaryGaitData();
    double findDesFootHeight(int footID);

private:
    double timestep_;  // Control loop timestep change
    UserCommand& user_cmd_;  // Command from user
    std::string trans_mode_; // Transition mode
    double height_;
    double g_;
    int trans_cntr_;     // Counter of the elapsed transition frames
    double foot_offset_;

    double Fr_;  // Froude number

    double dphase_;     // Phase change at each step
    double dphaseInit_; // Phase change at each step for current gait
    GaitData data_;     // Current gait data
    GaitData dataInit_; // Initial gait data before transition
    GaitData dataNext_; // The next gait to transition to
    
    // gait transition parameters
    bool gait_up;       // Transition gait up
    bool gait_down;     // Transition gait down
    bool gait_change;   // Transition to next gait
    bool stand_to_walk; // Transitioning from stand to walk
    bool trans_to_intermediate;
    double w1;          // Scalar of incremental phase of current gait
    double w2;          // Scalar of incremental phase of new gait
    double B;
    double trans_offset;
    double fr;
    double di;
    Eigen::Vector4d phaseTracker;
    Eigen::Vector4d phaseTarget;
    
    // foot trajectories stuff
    Eigen::MatrixXd legEndeffRelPos_, hipRelPos_;
    Eigen::Vector3d refBasePos_, curBasePos_;
    Eigen::Quaterniond refBaseQuat_, curBaseQuat_;
    Eigen::Matrix3d refBaseOrn_, curBaseOrn_, curBaseOrnZ_;
    Eigen::Vector3d refBaseRPY_, curBaseRPY_;
    Eigen::Vector3d curBaseLinVelLcl_, curBaseLinVelWrld_, refBaseLinVelWrld_;
    Eigen::Vector3d curBaseAngVelLcl_, curBaseAngVelWrld_, refBaseAngVelWrld_;
    std::vector<FootSwingTraj> footTrajs_;
    std::vector<FootRefs> footRefs_;
    double clearanceRatio_, footDeltaXLimit_, footDeltaYLimit_, raibertKp_;
    bool swingFootTrajInitFlag_;
};

}  // end bio_gait namespace
