#include "learning_to_adapt/gait_scheduler.hpp"

#include <cmath>
#include <iostream>

namespace bio_gait {

const double TOLERANCE = 1e-10;

Eigen::Vector3d matrixToRPY(const Eigen::Matrix3d& R) {
    // assert(R.isUnitary() && "R is not a unitary matrix");
    Eigen::Vector3d rpy = R.eulerAngles(2,1,0).reverse();

    if (rpy[1] < - M_PI/2)
        rpy[1] += 2 * M_PI;
    
    if (rpy[1] > M_PI/2) {
        rpy[1] = M_PI - rpy[1];
        if (rpy[0] < 0.)
            rpy[0] += M_PI;
        else
            rpy[0] -= M_PI;
        rpy[2] -= M_PI;
    }

    return rpy;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    // Convert vector to skew-symmetric matrix
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    M << 0, -v[2], v[1],
         v[2], 0, -v[0], 
        -v[1], v[0], 0;
    return M;
}

Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w) {
    // Computes the vectorized exponential map for SO(3)
    Eigen::Matrix3d A = skew(w);
    double theta = w.norm();
    if (theta < TOLERANCE) {
        return Eigen::Matrix3d::Identity();
    } 
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + (std::sin(theta)/theta)*A + 
        ((1-std::cos(theta))/(theta*theta))*A*A;
    return R;
}

GaitScheduler::GaitScheduler(const double timestep, UserCommand& user_cmd, std::string trans_mode, const double height, const double foot_offset)
    : timestep_(timestep),
      user_cmd_(user_cmd),
      trans_mode_(trans_mode),
      height_(height),
      g_(9.81),
      trans_cntr_(0),
      foot_offset_(foot_offset)
{
    // Start the gait in standing since we use this the most
    data_.current_gait = GaitType::STAND;
    // Set all gait data to zero
    data_.setZero();
    dataInit_.setZero();
    dataNext_.setZero();
    gait_up = false;
    gait_down = false;  
    gait_change = false;
    trans_to_intermediate = false;
    swingFootTrajInitFlag_ = false;
    // Create the gait from the nominal initial
    this->createGait();
    this->modifyGait();
    B=0;
    fr=0;
    phaseTracker.setZero();
    phaseTarget.setZero();
}

const GaitData&  GaitScheduler::gaitData() const { return data_; }
GaitData& GaitScheduler::gaitData() { return data_; }

void GaitScheduler::setNominalHeight(const double h) { height_ = h; }

void GaitScheduler::setupSwingFootTrajGen(const double clearance_ratio, const double foot_delta_x_limit, const double foot_delta_y_limit, const double raibert_kp, const Eigen::MatrixXd leg_endeff_rel_pos, const Eigen::MatrixXd hip_rel_pos, const Vector3d ref_base_pos, const Eigen::Vector4d ref_base_quat) {
    clearanceRatio_ = clearance_ratio;
    footDeltaXLimit_ = foot_delta_x_limit;
    footDeltaYLimit_ = foot_delta_y_limit;
    raibertKp_ = raibert_kp;
    legEndeffRelPos_.resize(4,3);
    legEndeffRelPos_(0,0) = leg_endeff_rel_pos(0,0);
    legEndeffRelPos_(0,1) = leg_endeff_rel_pos(0,1);
    legEndeffRelPos_(0,2) = leg_endeff_rel_pos(0,2);
    legEndeffRelPos_(1,0) = leg_endeff_rel_pos(1,0);
    legEndeffRelPos_(1,1) = leg_endeff_rel_pos(1,1);
    legEndeffRelPos_(1,2) = leg_endeff_rel_pos(1,2);
    legEndeffRelPos_(2,0) = leg_endeff_rel_pos(2,0);
    legEndeffRelPos_(2,1) = leg_endeff_rel_pos(2,1);
    legEndeffRelPos_(2,2) = leg_endeff_rel_pos(2,2);
    legEndeffRelPos_(3,0) = leg_endeff_rel_pos(3,0);
    legEndeffRelPos_(3,1) = leg_endeff_rel_pos(3,1);
    legEndeffRelPos_(3,2) = leg_endeff_rel_pos(3,2);
    hipRelPos_.resize(4,3);
    hipRelPos_(0,0) = hip_rel_pos(0,0);
    hipRelPos_(0,1) = hip_rel_pos(0,1);
    hipRelPos_(0,2) = hip_rel_pos(0,2);
    hipRelPos_(1,0) = hip_rel_pos(1,0);
    hipRelPos_(1,1) = hip_rel_pos(1,1);
    hipRelPos_(1,2) = hip_rel_pos(1,2);
    hipRelPos_(2,0) = hip_rel_pos(2,0);
    hipRelPos_(2,1) = hip_rel_pos(2,1);
    hipRelPos_(2,2) = hip_rel_pos(2,2);
    hipRelPos_(3,0) = hip_rel_pos(3,0);
    hipRelPos_(3,1) = hip_rel_pos(3,1);
    hipRelPos_(3,2) = hip_rel_pos(3,2);
    refBasePos_[0] = ref_base_pos[0];
    refBasePos_[1] = ref_base_pos[1];
    refBasePos_[2] = ref_base_pos[2];
    refBaseQuat_ = Eigen::Quaterniond(ref_base_quat[3], ref_base_quat[0], ref_base_quat[1], ref_base_quat[2]); // wxyz
    refBaseOrn_ = refBaseQuat_.toRotationMatrix();
    refBaseRPY_ = matrixToRPY(refBaseOrn_);
    refBaseLinVelWrld_.setZero();
    refBaseAngVelWrld_.setZero();
    footTrajs_.resize(4);
    footRefs_.resize(4);
    for (int i=0;i<4;i++) {
        footTrajs_[i] = FootSwingTraj();
        footTrajs_[i].setStartPosition(refBasePos_ + refBaseOrn_*legEndeffRelPos_.row(i).transpose());
        footTrajs_[i].setHeight(clearanceRatio_*refBasePos_(2));
        footTrajs_[i].setEndPosition(refBasePos_+refBaseOrn_*legEndeffRelPos_.row(i).transpose());
        footRefs_[i].position = refBasePos_ + refBaseOrn_*legEndeffRelPos_.row(i).transpose();
        footRefs_[i].positionLcl = legEndeffRelPos_.row(i);
    }
    
    swingFootTrajInitFlag_ = true;
}

void GaitScheduler::step(ConstRefVector3d lin_vel, ConstRefVector3d ang_vel, const Eigen::Ref<const Eigen::VectorXd>& base_pose, const Eigen::Ref<const Eigen::VectorXd>& base_vel, const Eigen::Ref<const Eigen::VectorXd>& foot_heights)
{
    //std::cout << "lin_vel: \n" << lin_vel << std::endl;
    //std::cout << "ang_vel: \n" << ang_vel << std::endl;
    
    if (lin_vel[0] > 0) {
        fr = pow(lin_vel[0],2)/(g_*height_);
    }
    else{
        fr = pow(ang_vel[2],2)/(g_*height_);
    }
    
    if (trans_mode_ == "STATIC") {
        this->modifyGait();
    }
    
    data_.froudeCmd_[0] = pow(lin_vel[0],2)/(g_*height_);
    data_.froudeCmd_[1] = pow(lin_vel[1],2)/(g_*height_);
    data_.froudeCmd_[2] = pow(ang_vel[2],2)/(g_*height_);
    
    // Check if gait should be changed depending on the mode
    if (trans_mode_ == "HARD"){
        fr = pow(lin_vel[0],2)/(g_*height_);
        if (fr > data_.fr_bounds[1]){
            user_cmd_.gait_type = user_cmd_.gait_type + 1;
            std::cout << "trigger up\n" << std::endl;
        }
        if (fr < data_.fr_bounds[0]){
            user_cmd_.gait_type = user_cmd_.gait_type - 1;
            std::cout << "trigger down\n" << std::endl;
        }
        // Modify the gait with settings
        this->modifyGait();
    }
    
    if (trans_mode_ == "SYNC"){
        fr = pow(lin_vel[0],2)/(g_*height_);
        if (fr > data_.fr_bounds[1]){
            this->createNextGait(GaitType(user_cmd_.gait_type + 1));
            this->calcTransParams();
            B = data_.B;
            phaseTarget = dataNext_.phase_offset-dataInit_.phase_offset;
            gait_change = true;
            di = round(1.2);
            trans_cntr_ = trans_cntr_ + di;
            data_.trans_flag = 1;
            if (trans_cntr_ > data_.B and data_.gait_name == "STAND"){
                stand_to_walk = true;
                data_.trans_flag = 0;
                std::cout << "\nTransition finished! " << di << " " << B << std::endl;
                user_cmd_.gait_type = user_cmd_.gait_type + 1;
                this->modifyGait();
                gait_change = false;
                trans_cntr_ = 0;
                phaseTracker.setZero();
                phaseTarget.setZero();
            }
        }
        
        if (fr < data_.fr_bounds[0]){
            this->createNextGait(GaitType(user_cmd_.gait_type - 1));
            this->calcTransParams();
            B = data_.B;
            phaseTarget = dataNext_.phase_offset-dataInit_.phase_offset;
            gait_change = true;
            di = round(1.2);
            trans_cntr_ = trans_cntr_ + di;
            data_.trans_flag = 1;
        }
    }
    
    if (trans_mode_ == "BIOSYNC"){
        if (lin_vel[0] > 0) {
            fr = pow(lin_vel[0],2)/(g_*height_);
        }
        else{
            fr = pow(ang_vel[2],2)/(g_*height_);
        }
        
        if (fr > data_.fr_bounds[1]){
            this->createNextGait(GaitType(user_cmd_.gait_type + 1));
            this->calcTransParams();
            B = data_.B;
            phaseTarget = dataNext_.phase_offset-dataInit_.phase_offset;
            gait_change = true;
            //trans_cntr_ ++;
            di = round(std::fmin(1 + (((fr-dataNext_.fr_bounds[0])/(dataNext_.fr_bounds[1]-dataNext_.fr_bounds[0]))*B), B));
            if (di <= 0) {
                di = 1;
            }
            // std::cout << di << std::endl;
            trans_cntr_ = trans_cntr_ + di;
            data_.trans_flag = 1;
            if (trans_cntr_ > data_.B and data_.gait_name == "STAND"){
                stand_to_walk = true;
                data_.trans_flag = 0;
                std::cout << "\nTransition finished! " << di << " " << B << std::endl;
                user_cmd_.gait_type = user_cmd_.gait_type + 1;
                this->modifyGait();
                gait_change = false;
                trans_cntr_ = 0;
                phaseTracker.setZero();
                phaseTarget.setZero();
            }
        }
        
        if (fr < data_.fr_bounds[0]){
            this->createNextGait(GaitType(user_cmd_.gait_type - 1));
            this->calcTransParams();
            B = data_.B;
            phaseTarget = dataNext_.phase_offset-dataInit_.phase_offset;
            gait_change = true;
            //trans_cntr_ ++;
            di = round(std::fmin(1 + (((dataNext_.fr_bounds[1]-fr)/(dataNext_.fr_bounds[1]-dataNext_.fr_bounds[0]))*B), B));
            if (di <= 0) {
                di = 1;
            }
            trans_cntr_ = trans_cntr_ + di;
            data_.trans_flag = 1;
        }
    }
    
    // static bio gait transition
    if (data_.current_gait != GaitType(user_cmd_.gait_type) and trans_mode_ == "BIOSYNCSTATIC") {
        //if (data_.gait_name != "INTERMEDIATEGAIT") {
        //    this->createNextGait(GaitType(10));
        //}
        //else {
            this->createNextGait(GaitType(user_cmd_.gait_type));
        //}
        this->calcTransParams();
        B = data_.B;
        phaseTarget = dataNext_.phase_offset-dataInit_.phase_offset;
        gait_change = true;
        //di = 1.0;
        float stab, stabNext;
        stab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
        stabNext = g_/(height_*pow(1/(dataNext_.period_time_nominal*dataNext_.switching_phase_nominal),2));
        di = round(std::fmin(1 + ((stab/stabNext)), B));
        if (di <= 0) {
            di = 1;
        }
        trans_cntr_ = trans_cntr_ + di;
        data_.trans_flag = 1;
        if (trans_cntr_ > data_.B and data_.gait_name == "STAND"){
            stand_to_walk = true;
            data_.trans_flag = 0;
            std::cout << "\nNext gait is "<< dataNext_.gait_name << std::endl;
            std::cout << "\nTransition finished! " << di << " " << B << std::endl;
            this->modifyGait();
            gait_change = false;
            trans_cntr_ = 0;
            phaseTracker.setZero();
            phaseTarget.setZero();
        }
    }
    
    // If gait transition required calculated blending weights
    if (gait_change == true) {
        w1 = 1 - (trans_cntr_/B);
        w2 = trans_cntr_/B;
        for (std::size_t idx = 0; idx < kNumLegs; idx++) {
            // The scaled period time for each leg
            data_.period_time(idx) = w1*dataInit_.period_time(idx) + w2*dataNext_.period_time(idx);
            // Phase at which to switch the leg from stance to swing
            data_.switching_phase(idx) = w1*dataInit_.switching_phase(idx) + w2*dataNext_.switching_phase(idx);
            // Find the total stance time over the gait cycle
            data_.time_stance(idx) = data_.period_time(idx) * data_.switching_phase(idx);
            // Find the total swing time over the gait cycle
            data_.time_swing(idx) = data_.period_time(idx) * (1.0 - data_.switching_phase(idx));
        }
    }
    
    // Iterate over the legs
    for (std::size_t idx = 0; idx < kNumLegs; idx++) {
        // Set the previous contact state for the next timestep
        data_.contact_state_prev(idx) = data_.contact_state_scheduled(idx);

        if (data_.gait_enabled(idx) == 1) {
            // Monotonic time based phase incrementation
            if (data_.current_gait == GaitType::STAND and (gait_up == false or gait_change == false)) {
                // Don't increment the phase when in stand mode
                dphase_ = 0.0;
            }
            else if (gait_change == true and (trans_mode_ == "BIOSYNCSTATIC" or trans_mode_ == "BIOSYNC" or trans_mode_ == "SYNC")) {
                dphase_ = (w1*data_.dt + w2*dataNext_.dt);
            }
            else {
                dphase_ = data_.phase_scale(idx) * (timestep_/data_.period_time_nominal);
            }
            
            // adjust phase variable
            if (gait_change == true) {
                if (trans_cntr_ <= B or data_.gait_name == "STAND") {
                    data_.phase_variable(idx) = data_.phase_variable(idx) + (((dataNext_.phase_offset(idx)-dataInit_.phase_offset(idx))/B)*di);
                    data_.phase_variable(idx) = std::fmod((data_.phase_variable(idx) + dphase_), 1);
                    phaseTracker(idx) = phaseTracker(idx) + (((dataNext_.phase_offset(idx)-dataInit_.phase_offset(idx))/B)*di);
                }
                else {
                    data_.phase_variable(idx) = data_.phase_variable(idx) + (phaseTarget(idx) - phaseTracker(idx));
                    data_.phase_variable(idx) = std::fmod((data_.phase_variable(idx) + dphase_), 1);
                    phaseTracker(idx) = phaseTracker(idx) + (((dataNext_.phase_offset(idx)-dataInit_.phase_offset(idx))/B)*di);
                }
            }
            else{
                // Find each leg's current phase
                data_.phase_variable(idx) = std::fmod((data_.phase_variable(idx) + dphase_), 1); 
            }


            // Check the current contact state
            if (data_.phase_variable(idx) <= data_.switching_phase(idx)) {
                // Leg is scheduled to be in contact
                data_.contact_state_scheduled(idx) = 1;
                // Stance subphase calculation
                data_.phase_stance(idx) = data_.phase_variable(idx) / data_.switching_phase(idx);
                // Swing phase has not started since leg is in stance
                data_.phase_swing(idx) = 0.0;
                // Calculate the remaining time in stance
                data_.time_stance_remaining(idx) = data_.period_time(idx) * (
                    data_.switching_phase(idx) - data_.phase_variable(idx));
                // Leg is in stance, no swing time remaining
                
                
                data_.time_swing_remaining(idx) = 0.0;
                
                // As leg is in stance, target height is zero
                data_.des_foot_height(idx) = foot_offset_;

                // First contact signifies scheduled touchdown
                if (data_.contact_state_prev(idx) == 0) {
                    // Set the touchdown flag to 1
                    data_.touchdown_scheduled(idx) = 1;
                }
                else {
                    // Set the touchdown flag to 0
                    data_.touchdown_scheduled(idx) = 0;
                }
            }
            else {
                // Leg is not scheduled to be in contact
                data_.contact_state_scheduled(idx) = 0;
                // Stance phase has completed since leg is in swing
                data_.phase_stance(idx) = 1.0;
                // Swing subphase calculation
                data_.phase_swing(idx) = (data_.phase_variable(idx) - data_.switching_phase(idx)) / (
                    1.0 - data_.switching_phase(idx));
                // Leg is in swing, no stance time remaining
                data_.time_stance_remaining(idx) = 0.0;
                // Calculate the remaining time in swing
                data_.time_swing_remaining(idx) = data_.period_time(idx) * (1 - data_.phase_variable(idx));
                
                // Find the swing leg's desired height
                data_.des_foot_height(idx) = this->findDesFootHeight(idx);

                // First contact signifies scheduled touchdown
                if (data_.contact_state_prev(idx) == 1) {
                    // Set the liftoff flag to 1
                    data_.liftoff_scheduled(idx) = 1;
                }
                else {
                    // Set the liftoff flag to 0
                    data_.liftoff_scheduled(idx) = 0;
                }
            }
        }
        else {
            // Leg is not enabled
            data_.phase_variable(idx) = 0.0;
            // Leg is not scheduled to be in contact
            data_.contact_state_scheduled(idx) = 0;
        }
        
    }
    
    if (swingFootTrajInitFlag_) {
        // swing foot ref stuff
        curBasePos_ = base_pose.head(3);
        Eigen::Vector4d tempQuat = base_pose.segment(3,4);
        curBaseQuat_ = Eigen::Quaterniond(tempQuat(3), tempQuat(0), tempQuat(1), tempQuat(2));
        curBaseOrn_ = curBaseQuat_.toRotationMatrix();
        curBaseRPY_ = matrixToRPY(curBaseOrn_); // Z-Y-X
        curBaseLinVelLcl_ = base_vel.head(3);
        curBaseAngVelLcl_ = base_vel.segment(3,3);
        curBaseLinVelWrld_ = curBaseOrn_ * curBaseLinVelLcl_;
        curBaseAngVelWrld_ = curBaseOrn_ * curBaseAngVelLcl_;
        double yaw = curBaseRPY_(2);
        curBaseOrnZ_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        // compute body pose and velocity reference
        refBaseLinVelWrld_ = refBaseOrn_ * lin_vel;
        refBaseAngVelWrld_ = refBaseOrn_ * ang_vel;
        refBasePos_ += timestep_ * refBaseLinVelWrld_;
        refBaseOrn_ = ExpSO3(timestep_*refBaseAngVelWrld_) * refBaseOrn_;
        refBaseQuat_ = Eigen::Quaterniond(refBaseOrn_);
        refBaseRPY_ = matrixToRPY(refBaseOrn_);
        // record the state positions of the swing feet
        for (int i=0;i<4;i++) {
            if (data_.liftoff_scheduled(i) == 1) {
                footTrajs_[i].setStartPosition(footRefs_[i].position);
            }
        }
        // compute swing foot poses
        for (int i=0;i<4;i++) {
            if (data_.contact_state_scheduled(i) == 0) {
                Eigen::Vector3d twistingVec(-hipRelPos_.row(i).transpose()(1), hipRelPos_.row(i).transpose()(0), 0.);
                Eigen::Vector3d curHipLinVelLcl = curBaseLinVelLcl_ + curBaseAngVelLcl_(2)*twistingVec;
                Eigen::Vector3d desHipLinVelLcl = lin_vel + ang_vel(2)*twistingVec;
                double relPosX = curHipLinVelLcl(0) * data_.time_stance(i) / 2
                                   + raibertKp_ * (curHipLinVelLcl(0) - desHipLinVelLcl(0))
                                   + (0.5*curBasePos_[2]/9.81) * (curBaseLinVelLcl_(1)*ang_vel(2));
                double relPosY = curHipLinVelLcl(1) * data_.time_stance(i) / 2
                                   + raibertKp_ * (curHipLinVelLcl(1) - desHipLinVelLcl(1))
                                   + (0.5*curBasePos_[2]/9.81) * (curBaseLinVelLcl_(0)*ang_vel(2));
                relPosX = std::min(std::max(relPosX, -footDeltaXLimit_), footDeltaXLimit_);
                relPosY = std::min(std::max(relPosY, -footDeltaYLimit_), footDeltaYLimit_);
                
                Eigen::Vector3d desFootRelPos(relPosX, relPosY, -curBasePos_(2));
                desFootRelPos += hipRelPos_.row(i).transpose();
                Eigen::Vector3d desLegEndeffPos = curBasePos_ + curBaseOrnZ_*desFootRelPos;
                desLegEndeffPos(2) = foot_offset_;
                
                footTrajs_[i].setEndPosition(desLegEndeffPos);
                footTrajs_[i].computeSwingTrajectory(data_.phase_swing(i), data_.time_swing(i));
                
                footRefs_[i].position = footTrajs_[i].getPosition();
                footRefs_[i].positionLcl = (footTrajs_[i].getPosition() - curBasePos_);
                footRefs_[i].positionLcl[2] = footTrajs_[i].getPosition()[2];
                
                data_.ref_foot_x[i] = footRefs_[i].positionLcl[0];
                data_.ref_foot_y[i] = footRefs_[i].positionLcl[1];
                data_.ref_foot_z[i] = footRefs_[i].positionLcl[2];
            }
            else {
                footRefs_[i].position[2] = foot_heights[i]; //foot_offset_;
                footRefs_[i].positionLcl[2] = foot_heights[i]; //foot_offset_;
                if (data_.gait_name == "STAND") {
                    footRefs_[i].position[0] = (curBasePos_ + curBaseOrn_*legEndeffRelPos_.row(i).transpose())[0]; //framePosition_[0];
                    footRefs_[i].position[1] = (curBasePos_ + curBaseOrn_*legEndeffRelPos_.row(i).transpose())[1]; //framePosition_[1];
                    footRefs_[i].positionLcl[0] = legEndeffRelPos_.row(i)[0]; //(framePosition_[0] - curBasePos_[0]);
                    footRefs_[i].positionLcl[1] = legEndeffRelPos_.row(i)[1]; //(framePosition_[1] - curBasePos_[1]);
                }
                data_.ref_foot_x[i] = footRefs_[i].positionLcl[0];
                data_.ref_foot_y[i] = footRefs_[i].positionLcl[1];
                data_.ref_foot_z[i] = footRefs_[i].positionLcl[2];
            }
        }
    }
    
    // Check if transition is complete
    if (trans_mode_ == "SYNC"){
        if (fr > data_.fr_bounds[1]){
            if (trans_cntr_ > data_.B){
                if (data_.gait_name == "STAND"){
                    stand_to_walk = true;
                }
                else{
                    stand_to_walk = false;
                }
                data_.trans_flag = 0;
                std::cout << "\nTransition finished! " << di << " " << B << std::endl;
                user_cmd_.gait_type = user_cmd_.gait_type + 1;
                this->modifyGait();
                gait_change = false;
                trans_cntr_ = 0;
                phaseTracker.setZero();
                phaseTarget.setZero();
            }
        }
        
        if (fr < data_.fr_bounds[0]){
            if (trans_cntr_ > data_.B){
                if (data_.gait_name == "STATIC_WALK"){
                    stand_to_walk = true;
                }
                else{
                    stand_to_walk = false;
                }
                data_.trans_flag = 0;
                std::cout << "\nTransition finished! " << di << " " << B << std::endl;
                user_cmd_.gait_type = user_cmd_.gait_type - 1;
                this->modifyGait();
                gait_change = false;
                trans_cntr_ = 0;
                phaseTracker.setZero();
                phaseTarget.setZero();
            }
        }
    }
    
    if (trans_mode_ == "BIOSYNC"){
        if (fr > data_.fr_bounds[1]){
            if (trans_cntr_ > data_.B){
                if (data_.gait_name == "STAND"){
                    stand_to_walk = true;
                }
                else{
                    stand_to_walk = false;
                }
                data_.trans_flag = 0;
                std::cout << "\nTransition finished! " << di << " " << B << std::endl;
                user_cmd_.gait_type = user_cmd_.gait_type + 1;
                this->modifyGait();
                gait_change = false;
                trans_cntr_ = 0;
                phaseTracker.setZero();
                phaseTarget.setZero();
            }
        }
        
        if (fr < data_.fr_bounds[0]){
            if (trans_cntr_ > data_.B){
                if (data_.gait_name == "STATIC_WALK"){
                    stand_to_walk = true;
                }
                else{
                    stand_to_walk = false;
                }
                data_.trans_flag = 0;
                std::cout << "\nTransition finished! " << di << " " << B << std::endl;
                user_cmd_.gait_type = user_cmd_.gait_type - 1;
                this->modifyGait();
                gait_change = false;
                trans_cntr_ = 0;
                phaseTracker.setZero();
                phaseTarget.setZero();
            }
        }
    }
    
    if (trans_mode_ ==  "BIOSYNCSTATIC") {
        if (gait_change == true) {
            if (trans_cntr_ > data_.B) {
                if (dataNext_.gait_name == "STAND") {
                    stand_to_walk = true;
                }
                else {
                    stand_to_walk = false;
                }
                data_.trans_flag = 0;
                std::cout << "\nNext gait is "<< dataNext_.gait_name << std::endl;
                std::cout << "phase target: " << phaseTarget(0) << " " << phaseTarget(1) << " "  << phaseTarget(2) << " "  << phaseTarget(3) << std::endl;
                std::cout << "\nTransition finished! " << di << " " << B  << " " << data_.C << std::endl;
                this->modifyGait();
                gait_change = false;
                trans_cntr_ = 0;
                phaseTracker.setZero();
                phaseTarget.setZero();
            }
        }
    }
}

double GaitScheduler::findDesFootHeight(int footID) {
    double x, footHeight;
    x = (data_.time_swing_remaining(footID)/((1-data_.switching_phase(footID))*data_.period_time(footID)))*2;
    footHeight = (2*(1-x)*x*user_cmd_.gait_step_height) + (std::pow(x,2)*user_cmd_.gait_step_height) + foot_offset_;
    
    return footHeight;
}

void GaitScheduler::modifyGait()
{
    /*
    if (dataNext_.gait_name == "INTERMEDIATEGAIT" and data_.current_gait != GaitType(user_cmd_.gait_type)){
        data_.next_gait = GaitType(10);
        this->createGait();
    }
    
    else if (data_.current_gait != GaitType(user_cmd_.gait_type)) {
        data_.next_gait = GaitType(user_cmd_.gait_type);
        this->createGait();
    }
    */
    
    if (data_.current_gait != GaitType(user_cmd_.gait_type)) {
        data_.next_gait = GaitType(user_cmd_.gait_type);
        this->createGait();
    }
    
}

void GaitScheduler::createNextGait(GaitType gaitId) {
    switch (gaitId) {
        case GaitType::STAND:
            dataNext_.gait_name = "STAND";
            dataNext_.Cmax = 2;
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 1.0;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 1.0;
            dataNext_.phase_offset << 0., 0., 0., 0.;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.fr_bounds << 0, 0.0001;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;
            
        case GaitType::STATIC_WALK:
            dataNext_.gait_name = "STATIC_WALK";
            dataNext_.Cmax = 2;
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 1.0;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 0.8;
            dataNext_.phase_offset << 0.0, 0.75, 0.5, 0.25;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.fr_bounds << 0.0001, 0.0024;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;

        case GaitType::TROT_WALK:
            dataNext_.gait_name = "TROT_WALK";
            dataNext_.Cmax = 2;
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.5;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 0.6;
            dataNext_.phase_offset << 0.0, 0.5, 0.5, 0.0;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.fr_bounds << 0.0024, 0.152;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;
            
        case GaitType::TROT:
            dataNext_.gait_name = "TROT";
            dataNext_.Cmax = 2;
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.4;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 0.5;
            dataNext_.phase_offset << 0.0, 0.5, 0.5, 0.0;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.fr_bounds << 0.152, 0.74;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;
            
        case GaitType::TROT_RUN:
            dataNext_.gait_name = "TROT_RUN";
            dataNext_.Cmax = 2;
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.3;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 0.4;
            dataNext_.phase_offset << 0.0, 0.5, 0.5, 0.0;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.fr_bounds << 0.74, 1.5;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;

        case GaitType::ROTARY_GALLOP:
            dataNext_.gait_name = "ROTARY_GALLOP";
            dataNext_.Cmax = 2;
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.4;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 0.2;
            dataNext_.phase_offset << 0.0, 0.8571, 0.3571, 0.5;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.fr_bounds << 1.5, 10.0;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;

        case GaitType::PACE:
            dataNext_.gait_name = "PACE";
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.35;
            dataNext_.initial_phase = 0.25;
            dataNext_.switching_phase_nominal = 0.5;
            dataNext_.phase_offset << 0.0, 0.5, 0.0, 0.5;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;

        case GaitType::BOUND:
            dataNext_.gait_name = "BOUND";
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.4;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 0.4;
            dataNext_.phase_offset << 0.5, 0.5, 0.0, 0.0;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;

        case GaitType::TRAVERSE_GALLOP:
            dataNext_.gait_name = "TRAVERSE_GALLOP";
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.5;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 0.2;
            dataNext_.phase_offset << 0.0, 0.8571, 0.3571, 0.5;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;

        case GaitType::PRONK:
            dataNext_.gait_name = "PRONK";
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.5;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 0.5;
            dataNext_.phase_offset << 0.0, 0.0, 0.0, 0.0;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;
            
        case GaitType::INTERMEDIATEGAIT:
            dataNext_.gait_name = "INTERMEDIATEGAIT";
            dataNext_.Cmax = 2;
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.1;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 1.0;
            dataNext_.phase_offset << 0., 0., 0., 0.;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.fr_bounds << 0, 0.0001;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;
            
        case GaitType::UNNATURAL:
            dataNext_.gait_name = "UNNATURAL";
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.4;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 0.5;
            dataNext_.phase_offset << 0.5, 0.5, 0.5, 0.0;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;

        case GaitType::AMBLE:
            dataNext_.gait_name = "AMBLE";
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.5;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 0.6250;
            dataNext_.phase_offset << 0.0, 0.5, 0.25, 0.75;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;
            
        case GaitType::HOP:
            dataNext_.gait_name = "HOP";
            dataNext_.gait_enabled << 1, 1, 1, 1;
            dataNext_.period_time_nominal = 0.3;
            dataNext_.initial_phase = 0.0;
            dataNext_.switching_phase_nominal = 0.5;
            dataNext_.phase_offset << 0.0, 0.0, 0.0, 0.0;
            dataNext_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataNext_.overrideable = 1;
            dataNext_.L = dataNext_.period_time_nominal/timestep_;
            dataNext_.dt = 1/dataNext_.L;
            dataNext_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            break;

    }
    
    // Set the gait parameters for each leg
    for (std::size_t idx = 0; idx < kNumLegs; idx++) { 
        // GAIT NEXT
        // The scaled period time for each leg
        dataNext_.period_time(idx) = dataNext_.period_time_nominal / dataNext_.phase_scale(idx);
        // Phase at which to switch the leg from stance to swing
        dataNext_.switching_phase(idx) = dataNext_.switching_phase_nominal;
        // Initialize the phase variables according to offset
        dataNext_.phase_variable(idx) = dataNext_.initial_phase + dataNext_.phase_offset(idx);
        // Find the total stance time over the gait cycle
        dataNext_.time_stance(idx) = dataNext_.period_time(idx) * dataNext_.switching_phase(idx);
        // Find the total swing time over the gait cycle
        dataNext_.time_swing(idx) = dataNext_.period_time(idx) * (1.0 - dataNext_.switching_phase(idx));
    }
    //std::cout << "phase: " << dataNext_.phase_offset(0) << " " << dataNext_.phase_offset(1) << " "  << dataNext_.phase_offset(2) << " "  << dataNext_.phase_offset(3) << std::endl;
}

void GaitScheduler::calcTransParams() {
    float trans_coeff = 0.2;
    
    // Shift up gait transition parameters
    // linear
    if (trans_mode_ == "SYNC"){
        data_.B = ((data_.L + dataNext_.L)/2)*trans_coeff;
    }
    // bio
    if (trans_mode_ == "BIOSYNC"){
        data_.C = abs(1 - (fr/1.)) * data_.Cmax;
        data_.B = round(((data_.L + dataNext_.L)/2) * data_.C);
        
    }
    // bio static
    if (trans_mode_ == "BIOSYNCSTATIC"){
        //data_.C = abs(1 - (fr/1.)) * data_.Cmax;
        data_.C = exp(-2*fr);
        data_.B = round(((data_.L + dataNext_.L)/2) * data_.C);
    }
    if (data_.gait_name == "STAND" or dataNext_.gait_name == "STAND") {
        data_.B = 1;
    }
    //if (dataNext_.gait_name == "INTERMEDIATEGAIT") {
    //    data_.B = 0.1/0.002;
    //}
}

void GaitScheduler::createGait()
{
    std::cout << "[quad_gait/GaitScheduler::createGait] Transitioning gait from " 
        << data_.gait_name << " to ";
    // Case structure gets the appropriate parameters
    switch (data_.next_gait) {
        case GaitType::STAND:
            // Current gait data
            data_.gait_name = "STAND";
            data_.Cmax = 2;
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 1.0;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 1.0;
            data_.phase_offset << 0., 0., 0., 0.;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.vel_bounds << 0, 0.0009;
            data_.fr_bounds << 0, 0.0001;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            // Initial gait data before transition
            dataInit_.gait_name = "STAND";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 1.0;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 1.0;
            dataInit_.phase_offset << 0., 0., 0., 0.;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.vel_bounds << 0, 0.0009;
            dataInit_.fr_bounds << 0, 0.0001;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            // Shift up gait transition parameters
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;

            break;

        case GaitType::STATIC_WALK:
            // Current gait data
            data_.gait_name = "STATIC_WALK";
            data_.Cmax = 2;
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 1.0;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 0.8;
            data_.phase_offset << 0.0, 0.75, 0.5, 0.25;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.vel_bounds << 0.0009, 0.05;
            data_.fr_bounds << 0.0001, 0.0024;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            // Initial gait data before transition
            dataInit_.gait_name = "STATIC_WALK";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 1.0;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 0.8;
            dataInit_.phase_offset << 0.0, 0.75, 0.5, 0.25;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.vel_bounds << 0.0009, 0.05;
            dataInit_.fr_bounds << 0.0001, 0.0024;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            // Shift up gait transition parameters
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;
            
            break;

        case GaitType::TROT_WALK:
            // Current gait data
            data_.gait_name = "TROT_WALK";
            data_.Cmax = 2;
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.5;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 0.6;
            data_.phase_offset << 0.0, 0.5, 0.5, 0.0;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.vel_bounds << 0.05, 0.5;
            data_.fr_bounds << 0.0024, 0.152;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            // Initial gait data before transition
            dataInit_.gait_name = "TROT_WALK";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.5;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 0.6;
            dataInit_.phase_offset << 0.0, 0.5, 0.5, 0.0;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.vel_bounds << 0.05, 0.5;
            dataInit_.fr_bounds << 0.0024, 0.152;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            // Shift up gait transition parameters
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;
            
            break;

        case GaitType::TROT:
            // Current gait data
            data_.gait_name = "TROT";
            data_.Cmax = 2;
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.4;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 0.5;
            data_.phase_offset << 0.0, 0.5, 0.5, 0.0;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.vel_bounds << 0.5, 0.8;
            data_.fr_bounds << 0.152, 0.74;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            // Initial gait data before transition
            dataInit_.gait_name = "TROT";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.4;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 0.5;
            dataInit_.phase_offset << 0.0, 0.5, 0.5, 0.0;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.vel_bounds << 0.5, 0.8;
            dataInit_.fr_bounds << 0.152, 0.74;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            // Shift up gait transition parameters
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;
            
            break;

        case GaitType::TROT_RUN:
            // Current gait data
            data_.gait_name = "TROT_RUN";
            data_.Cmax = 2;
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.3;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 0.4;
            data_.phase_offset << 0.0, 0.5, 0.5, 0.0;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.vel_bounds << 0.8, 10;
            data_.fr_bounds << 0.74, 1.5;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            // Initial gait data before transition
            dataInit_.gait_name = "TROT_RUN";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.3;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 0.4;
            dataInit_.phase_offset << 0.0, 0.5, 0.5, 0.0;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.vel_bounds << 0.8, 10;
            dataInit_.fr_bounds << 0.74, 1.5;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));

            // Shift up gait transition parameters
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;
            
            break;

        case GaitType::ROTARY_GALLOP:
            // Current gait data
            data_.gait_name = "ROTARY_GALLOP";
            data_.Cmax = 2;
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.4;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 0.2;
            data_.phase_offset << 0.0, 0.8571, 0.3571, 0.5;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.vel_bounds << 1, 1.5;
            data_.fr_bounds << 1.5, 10.0;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            // Initial gait data before transition
            dataInit_.gait_name = "ROTARY_GALLOP";
            dataInit_.Cmax = 2;
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.4;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 0.2;
            dataInit_.phase_offset << 0.0, 0.8571, 0.3571, 0.5;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.vel_bounds << 1, 1.5;
            dataInit_.fr_bounds << 1.5, 10.0;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;

            break;

        case GaitType::PACE:
            data_.gait_name = "PACE";
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.35;
            data_.initial_phase = 0.25;
            data_.switching_phase_nominal = 0.5;
            data_.phase_offset << 0.0, 0.5, 0.0, 0.5;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            dataInit_.gait_name = "PACE";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.35;
            dataInit_.initial_phase = 0.25;
            dataInit_.switching_phase_nominal = 0.5;
            dataInit_.phase_offset << 0.0, 0.5, 0.0, 0.5;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            // Shift up gait transition parameters
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;
            break;

        case GaitType::BOUND:
            data_.gait_name = "BOUND";
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.4;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 0.4;
            data_.phase_offset << 0.5, 0.5, 0.0, 0.0;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            dataInit_.gait_name = "BOUND";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.4;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 0.4;
            dataInit_.phase_offset << 0.5, 0.5, 0.0, 0.0;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            // Shift up gait transition parameters
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;
            break;

        case GaitType::TRAVERSE_GALLOP:
            data_.gait_name = "TRAVERSE_GALLOP";
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.5;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 0.2;
            data_.phase_offset << 0.0, 0.8571, 0.3571, 0.5;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            dataInit_.gait_name = "TRAVERSE_GALLOP";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.5;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 0.2;
            dataInit_.phase_offset << 0.0, 0.8571, 0.3571, 0.5;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            // Shift up gait transition parameters
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;
            break;

        case GaitType::PRONK:
            data_.gait_name = "PRONK";
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.5;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 0.5;
            data_.phase_offset << 0.0, 0.0, 0.0, 0.0;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            dataInit_.gait_name = "PRONK";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.5;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 0.5;
            dataInit_.phase_offset << 0.0, 0.0, 0.0, 0.0;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            // Shift up gait transition parameters
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;
            break;
            
        case GaitType::INTERMEDIATEGAIT:
            // Current gait data
            data_.gait_name = "INTERMEDIATEGAIT";
            data_.Cmax = 2;
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.25;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 1.0;
            data_.phase_offset << 0., 0., 0., 0.;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            // Initial gait data before transition
            dataInit_.gait_name = "INTERMEDIATEGAIT";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.25;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 1.0;
            dataInit_.phase_offset << 0., 0., 0., 0.;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            // Shift up gait transition parameters
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;
            break;
            
        case GaitType::UNNATURAL:
            data_.gait_name = "UNNATURAL";
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.4;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 0.5;
            data_.phase_offset << 0.5, 0.5, 0.5, 0.0;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            dataInit_.gait_name = "UNNATURAL";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.4;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 0.5;
            dataInit_.phase_offset << 0.5, 0.5, 0.5, 0.0;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            data_.L = dataNext_.period_time_nominal/timestep_;
            data_.dt = 1/dataNext_.L;
            
            break;
           
        case GaitType::AMBLE:
            data_.gait_name = "AMBLE";
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.5;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 0.6250;
            data_.phase_offset << 0.0, 0.5, 0.25, 0.75;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            dataInit_.gait_name = "AMBLE";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.5;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 0.6250;
            dataInit_.phase_offset << 0.0, 0.5, 0.25, 0.75;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            
            data_.L = dataNext_.period_time_nominal/timestep_;
            data_.dt = 1/dataNext_.L;
            
            break;
            
        case GaitType::HOP:
            data_.gait_name = "HOP";
            data_.gait_enabled << 1, 1, 1, 1;
            data_.period_time_nominal = 0.3;
            data_.initial_phase = 0.0;
            data_.switching_phase_nominal = 0.5;
            data_.phase_offset << 0.0, 0.0, 0.0, 0.0;
            data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            data_.overrideable = 1;
            data_.FrStab = g_/(height_*pow(1/(data_.period_time_nominal*data_.switching_phase_nominal),2));
            
            dataInit_.gait_name = "HOP";
            dataInit_.gait_enabled << 1, 1, 1, 1;
            dataInit_.period_time_nominal = 0.3;
            dataInit_.initial_phase = 0.0;
            dataInit_.switching_phase_nominal = 0.5;
            dataInit_.phase_offset << 0.0, 0.0, 0.0, 0.0;
            dataInit_.phase_scale << 1.0, 1.0, 1.0, 1.0;
            dataInit_.overrideable = 1;
            dataInit_.FrStab = g_/(height_*pow(1/(dataInit_.period_time_nominal*dataInit_.switching_phase_nominal),2));
            
            // Shift up gait transition parameters
            data_.L = data_.period_time_nominal/timestep_;
            data_.dt = 1/data_.L;
            
            break;
            
    }

    // Gait has switched
    data_.current_gait = data_.next_gait;

    std::cout << data_.gait_name << "\n" << std::endl;

    // Calculate the auxilliary gait information
    this->calcAuxiliaryGaitData();
    std::cout << "phase: " << data_.phase_variable(0) << " " << data_.phase_variable(1) << " "  << data_.phase_variable(2) << " "  << data_.phase_variable(3) << std::endl;
}

void GaitScheduler::calcAuxiliaryGaitData()
{
    // Set the gait parameters for each leg
    for (std::size_t idx = 0; idx < kNumLegs; idx++) {
        if (data_.gait_enabled(idx) == 1) {
            // CURRENT GAIT
            // The scaled period time for each leg
            data_.period_time(idx) = data_.period_time_nominal / data_.phase_scale(idx);
            // Phase at which to switch the leg from stance to swing
            data_.switching_phase(idx) = data_.switching_phase_nominal;
            if (stand_to_walk == true or trans_mode_ == "HARD" or trans_mode_ == "STATIC"){
                // Initialize the phase variables according to offset
                data_.phase_variable(idx) = data_.initial_phase + data_.phase_offset(idx);
            }
            // Find the total stance time over the gait cycle
            data_.time_stance(idx) = data_.period_time(idx) * data_.switching_phase(idx);
            // Find the total swing time over the gait cycle
            data_.time_swing(idx) = data_.period_time(idx) * (1.0 - data_.switching_phase(idx));            
            
            // INIT GAIT
            // The scaled period time for each leg
            dataInit_.period_time(idx) = dataInit_.period_time_nominal / dataInit_.phase_scale(idx);
            // Phase at which to switch the leg from stance to swing
            dataInit_.switching_phase(idx) = dataInit_.switching_phase_nominal;
            // Initialize the phase variables according to offset
            dataInit_.phase_variable(idx) = dataInit_.initial_phase + dataInit_.phase_offset(idx);
            // Find the total stance time over the gait cycle
            dataInit_.time_stance(idx) = dataInit_.period_time(idx) * dataInit_.switching_phase(idx);
            // Find the total swing time over the gait cycle
            dataInit_.time_swing(idx) = dataInit_.period_time(idx) * (1.0 - dataInit_.switching_phase(idx));
        }
        else {
            // The scaled period time for each leg
            data_.period_time(idx) = 0.0;
            // Phase at which to switch the leg from stance to swing
            data_.switching_phase(idx) = 0.0;
            // Initialize the phase variables according to offset
            data_.phase_variable(idx) = 0.0;
            // Leg is never in stance
            data_.time_stance(idx) = 0.0;
            // Leg is always in "swing"
            data_.time_swing(idx) = 1.0 / data_.period_time(idx);
        }
    }
}

}  // end bio_gait namespace
