import os
import sys
import inspect
import time
import signal
import math
import numpy as np
import torch.nn as nn

from learning_to_adapt.yaml_parser import load_yaml
from learning_to_adapt.engine_interface import EngineInterface
from learning_to_adapt.policy_manager import PolicyManager
from learning_to_adapt.obs_manager import ObsManager
from learning_to_adapt_pywrap import UserCommand, GaitScheduler

# absolute directory of this package
rootdir = os.path.dirname(os.path.dirname(
        os.path.abspath(inspect.getfile(inspect.currentframe()))))
        
def main(argv):
    # Load configuration file
    if len(argv) == 1:
        cfg_file = argv[0]
    else:
        raise RuntimeError("Usage: python3 src/learning_to_adapt/scripts/run_demo.py /<config file within root folder>")
    input_streams = [sys.stdin]
    configs = load_yaml(rootdir + cfg_file)
    
    robot_path= rootdir + "/models" + configs["models"]["robot"]
    ground_path = rootdir + "/models" + configs["models"]["ground"]
    simulation_dt = configs["time_steps"]["simulation"]
    control_ll_dt = configs["time_steps"]["control_ll"]
    loco_policy_dt = configs["time_steps"]["loco_policy"]
    gs_policy_dt = configs["time_steps"]["gs_policy"]
    mode = configs["engine"]
    trq_lims = configs["robot"]["joint_torque_limits"]
    max_steps = int(300/simulation_dt)
    
    # setup control freq management params
    control_ll_count = control_ll_dt/simulation_dt
    loco_policy_count = loco_policy_dt/simulation_dt
    gs_policy_count = gs_policy_dt/simulation_dt
    
    # set up simulation
    sim = EngineInterface(
        config = configs,
        model_path_ground = ground_path,
        model_path_robot = robot_path
    )
    sim.add_robot(configs, robot_path, 0, 0)
    sim.initialise_server()
    
    # Create a gait scheduler
    user_cmd = UserCommand()
    user_cmd.gait_type = 0
    user_cmd.gait_override = 1
    user_cmd.gait_step_height = 0.075
    height = configs["bio_gait"]["leg_length"]
    foot_offset = configs["bio_gait"]["foot_offset"]
    gait_mode = configs["bio_gait"]["mode"]
    clearance_ratio = configs["bio_gait"]["clearance_ratio"]
    foot_delta_x_limit = configs["bio_gait"]["foot_delta_x_limit"]
    foot_delta_y_limit = configs["bio_gait"]["foot_delta_y_limit"]
    raibert_kp = configs["bio_gait"]["raibert_kp"]
    init_base_pos = configs["bio_gait"]["init_base_pos"]
    init_base_quat = configs["bio_gait"]["init_base_quat"]
    leg_endeff_rel_pos = configs["bio_gait"]["leg_endeff_rel_pos"]
    hip_rel_pos = configs["bio_gait"]["hip_rel_pos"]
    
    # set up loco policy
    obs_manager = ObsManager()
    loco_configs = configs["loco_policy"]
    obs_structure_loco = loco_configs["obs_structure"]
    weights_path_loco = rootdir + loco_configs["policy_path"] + loco_configs["weights_name"]
    mean_path_loco = rootdir + loco_configs["policy_path"] + loco_configs["mean_name"]
    var_path_loco = rootdir + loco_configs["policy_path"] + loco_configs["var_name"]
    ob_dim_loco = loco_configs["ob_dim"]
    act_dim_loco = loco_configs["act_dim"]
    action_mean_loco = loco_configs["gc"]
    action_std_loco = loco_configs["action_std"]
    policy_net_loco = loco_configs["policy_net"]
    loco_policy = PolicyManager(weights_path=weights_path_loco,
                                   mean_path=mean_path_loco,
                                   var_path=var_path_loco,
                                   ob_dim=ob_dim_loco,
                                   act_dim=act_dim_loco,
                                   action_mean=action_mean_loco,
                                   action_std=action_std_loco,
                                   policy_net=policy_net_loco)
    
    # set up gait selection policy
    gs_configs = configs["gait_selection_policy"]
    obs_structure_gs = gs_configs["obs_structure"]
    weights_path_gs = rootdir + gs_configs["policy_path"] + gs_configs["weights_name"]
    mean_path_gs = rootdir + gs_configs["policy_path"] + gs_configs["mean_name"]
    var_path_gs = rootdir + gs_configs["policy_path"] + gs_configs["var_name"]
    ob_dim_gs = gs_configs["ob_dim"]
    act_dim_gs = gs_configs["act_dim"]
    action_mean_gs = gs_configs["action_mean"]
    action_std_gs = gs_configs["action_std"]
    policy_net_gs = gs_configs["policy_net"]
    gs_policy = PolicyManager(weights_path=weights_path_gs,
                               mean_path=mean_path_gs,
                               var_path=var_path_gs,
                               ob_dim=ob_dim_gs,
                               act_dim=act_dim_gs,
                               action_mean=action_mean_gs,
                               action_std=action_std_gs,
                               policy_net=policy_net_gs,
                               activation=nn.Tanh,
                               action_type='disc')
                               
    # set up early cancel
    def handler(*args):
        sim.shutdown()
        exit(1)
    signal.signal(signal.SIGINT, handler)
    
    # set up containers
    obs_loco = np.zeros(configs["loco_policy"]["ob_dim"])
    obs_gs = np.zeros(configs["gait_selection_policy"]["ob_dim"])
    active_gaits = [0, 3, 4, 7, 9, 11, 12, 13, 3, 42]
    gait_id = 0
    manual_gait_id = 0
    lin_vel_cmd = np.array([0.0, 0.0, 0.0])
    ang_vel_cmd = np.array([0.0, 0.0, 0.0])
    vel_cmd = np.array([0.0, 0.0, 0.0])
    kill_flag = False
    kp = np.ones(len(configs['robot']['nominal_joint_configuration']))*configs['robot']['joint_kp']
    kd = np.ones(len(configs['robot']['nominal_joint_configuration']))*configs['robot']['joint_kd']
    foot_frame_names = configs['robot']['limb_endeff_link_names']
    time.sleep(5)
    
    # TODO: clean up
    sim.create_tracking_camera()
    time.sleep(1)
    start_time = sim.get_time_since_start()
    time.sleep(control_ll_dt)
    current_time = start_time
    track_time = start_time
    new_vel = 0
    loop_count = 0
    last_control_time = time.time()
    last_action_time = time.time()
    last_command_time = time.time()
    policy_action_loco = np.array(configs['robot']['nominal_joint_configuration'])
    policy_action_gs = 0
    current_time = 0
    sim.reset()
    
    # set gait id
    manual_gait_id = 9
    user_cmd = UserCommand()
    user_cmd.gait_type = user_cmd.gait_type = active_gaits[manual_gait_id]
    user_cmd.gait_override = 1
    user_cmd.gait_step_height = 0.075
    if manual_gait_id == 9:
        BGS_mode = "BIOSYNCSTATIC"
    else:
        BGS_mode = "STATIC"
    scheduler = GaitScheduler(control_ll_dt, user_cmd, BGS_mode, height, foot_offset)
    scheduler.setupSwingFootTrajGen(clearance_ratio,
                                    foot_delta_x_limit,
                                    foot_delta_y_limit,
                                    raibert_kp,
                                    leg_endeff_rel_pos,
                                    hip_rel_pos,
                                    init_base_pos,
                                    init_base_quat)
    
    # distance constrained velocity cmd
    exp_distance = 20 #20 m
    exp_max_x_vel = 2 #3 m/s
    exp_max_yaw_vel = 0
    exp_omega = (2*exp_max_x_vel)/exp_distance
    exp_total_time = (np.pi*exp_distance)/(2*exp_max_x_vel)
    exp_total_steps = (np.pi*exp_distance)/(2*exp_max_x_vel*control_ll_dt)
    exp_curr_step = 0
    yaw_theta = 0
    yaw_dtheta = (2*np.pi)/exp_total_steps
    
    # TODO: cleanup 
    prev_gait = "STAND"
    stand_counter = 0
    est_reset = False
    prevVelCmdGS = np.zeros(3)
    accVelCmdGS = np.zeros(3)
    prevVelCmdDC = np.zeros(3)
    accVelCmdDC = np.zeros(3)
    dEnergySum = 0
    WEXT = 0
    prevEnergyK = sim.get_kinetic_energy()
    prevEnergyP = sim.get_potential_energy()
    cmd_err_mean = 0
    CoT = 0
    musculoskeletal = 0
    cont_err = 0
    cont_forces = 0
    stride_time = [0,0,0,0]
    stride_time_data = [0,0,0,0]
    curr_foot_contact = [1,1,1,1]
    prev_foot_contact = [1,1,1,1]
    
    start_time = time.time()
    
    for steps in range(max_steps):
        loop_start_time = time.time()
        # collect observations
        sim.update_robot_states()
        
        if kill_flag == True:
            break
        
        if loop_count % control_ll_count == 0:
            # collect sensor observations
            last_control_time = time.time()
            imu_ang_vel = sim.get_raw_base_imu_angular_velocity(0)
            imu_lin_acc = sim.get_raw_base_imu_linear_acceleration(0)
            joint_pos = sim.get_raw_joint_positions(0)
            joint_vel = sim.get_raw_joint_velocities(0)
            joint_trq = sim.get_raw_joint_efforts(0)
            cont_frc = sim.get_raw_limb_contact_forces(0)
            
            # update velociy cmd
            
            if current_time <= 1:
                vel_cmd[0] = 0
                vel_cmd[2] = 0
                if manual_gait_id != 9:
                    user_cmd.gait_type = 0
            elif current_time > 1 and current_time <= (exp_total_time+1):
                exp_curr_step += 1
                yaw_theta = yaw_theta + yaw_dtheta
                vel_cmd[0] = exp_max_x_vel*math.sin(exp_omega*((time.time()-start_time)-1))
                vel_cmd[2] = exp_max_yaw_vel*math.sin(yaw_theta)
                if manual_gait_id != 9:
                    user_cmd.gait_type = active_gaits[manual_gait_id]
            elif current_time > (exp_total_time+1) and current_time <= (exp_total_time+2):
                vel_cmd[0] = 0
                vel_cmd[2] = 0
                if manual_gait_id != 9:
                    user_cmd.gait_type = 0
            else:
                break
            des_lin_vel = np.array([vel_cmd[0], vel_cmd[1], 0.0])
            des_ang_vel = np.array([0.0, 0.0, vel_cmd[2]])
            
            # update gait
            base_pose = np.concatenate((sim.get_base_position(0), sim.get_base_quaternion(0)))
            base_velocity = np.concatenate((sim.get_base_linear_velocity(0), sim.get_base_angular_velocity(0)))
            foot_heights = np.zeros(4)
            foot_heights[0] = sim.get_link_position(foot_frame_names[0])[2]
            foot_heights[1] = sim.get_link_position(foot_frame_names[1])[2]
            foot_heights[2] = sim.get_link_position(foot_frame_names[2])[2]
            foot_heights[3] = sim.get_link_position(foot_frame_names[3])[2]
            scheduler.step(des_lin_vel, des_ang_vel, base_pose, base_velocity, foot_heights)
            
        joint_trq_sat = joint_trq/trq_lims
        obs_manager.updateAllObs(sim, scheduler, vel_cmd, joint_trq_sat)
        
        # get loco action from obs
        if loop_count % loco_policy_count == 0:
            last_action_time = time.time()
            obs_loco = obs_manager.generateObsArray(obs_structure_loco)
            policy_action_loco = loco_policy.get_action(obs_loco)
            
        # get gc action from obs
        if loop_count % gs_policy_count == 0 and loop_count != 0:
            accVelCmdGS = (vel_cmd - prevVelCmdGS)/gs_policy_dt
            prevVelCmdGS = np.copy(vel_cmd)
            obs_manager.updateSpecificOb("accCmd", accVelCmdGS)
            obs_manager.updateSpecificOb("gsAction", policy_action_gs)
            obs_gs = obs_manager.generateObsArray(obs_structure_gs)
            policy_action_gs = int(gs_policy.get_action(obs_gs))
            if scheduler.gaitData().trans_flag != 1 and active_gaits[manual_gait_id] == 42:
                user_cmd.gait_type = active_gaits[policy_action_gs]
            
        # send action to robot
        sim.set_next_action(policy_action_loco)
        sim.step(policy_action_loco, update_states=False, sleep=False)
        loop_end_time = time.time()
        while loop_end_time-loop_start_time < simulation_dt:
            loop_end_time = time.time()
        current_time = sim.get_time_since_start()
        track_time = track_time + simulation_dt
        
        loop_count = loop_count + 1

    # shutdown server
    sim.shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
