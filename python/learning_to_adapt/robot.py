import numpy as np
from numpy import linalg as LA
import raisimpy as raisim
import time
from scipy.spatial.transform import Rotation as R

from learning_to_adapt.joint_controller_simple_PD import JointController

class Robot:
    def __init__(self, cfg, robot, robot_id, x, y):
        # set robot
        self.robot = robot
        self.robot_id = robot_id
        
        # indexs, names and orders
        self.ctrl_mode = cfg['robot']['ctrl_mode']
        self.joint_order = cfg['robot']['joint_order']
        self.limb_endeff_names = cfg['robot']['limb_endeff_names']
        self.robot.setName(cfg["robot"]["name"])
        self.base_name = cfg['robot']['base_name']
        self.footIndx = []
        self.footIndx.append(self.robot.getBodyIdx(self.limb_endeff_names[0]))
        self.footIndx.append(self.robot.getBodyIdx(self.limb_endeff_names[1]))
        self.footIndx.append(self.robot.getBodyIdx(self.limb_endeff_names[2]))
        self.footIndx.append(self.robot.getBodyIdx(self.limb_endeff_names[3]))
        
        # fetch robot data
        self.gcDim = self.robot.getGeneralizedCoordinateDim()
        self.gvDim = self.robot.getDOF()
        if cfg['robot']['fixed_base'] == True:
            self.nJoints = self.gvDim
        else:
            self.nJoints = self.gvDim - 6
        
        # set up containers
        self.gc = np.zeros(self.gcDim)
        self.gv = np.zeros(self.gvDim)
        self.gf = np.zeros(self.gvDim)
        self.gc_init = np.array(cfg['robot']['nominal_base_position']+cfg['robot']['nominal_base_orientation']+cfg['robot']['nominal_joint_configuration'])
        self.gc_init[0] = x
        self.gc_init[1] = y
        self.gv_init = np.zeros(self.gvDim)
        self.contact_forces = np.zeros(len(self.limb_endeff_names))
        self.normal_contact_forces = np.zeros(len(self.limb_endeff_names))
        self.contact_forces_magnitude = np.zeros(len(self.limb_endeff_names))
        self.base_lin_vel_prev = None
        self.base_ang_vel_prev = None
        self.base_lin_acc = np.zeros(3)
        self.base_ang_acc = np.zeros(3)
        self.g = np.array([0., 0., -9.81])
        
        # set up controller
        if self.ctrl_mode == "simple_PD_control":
            self.des_action = np.zeros(self.robot.getDOF())
            self.joint_torque_limits = cfg['robot']['joint_torque_limits']
            self.jointKp = np.ones(len(cfg['robot']['nominal_joint_configuration']))*cfg['robot']['joint_kp']
            self.jointKd = np.ones(len(cfg['robot']['nominal_joint_configuration']))*cfg['robot']['joint_kd']
            self.robot.setPdGains(
                np.zeros(self.robot.getDOF()), np.zeros(self.robot.getDOF()))
            self.joint_controller = JointController(self.joint_order, self.joint_torque_limits)
        else:
            raise ValueError(f"Control mode {self.ctrl_mode} is not supported.")
        
        # set up IMU data
        self.rng = np.random.default_rng()
        self.base_imu_acc_bias = np.zeros(3)
        self.base_imu_gyro_bias = np.zeros(3)
        self.base_imu_acc_thermal = np.zeros(3)
        self.base_imu_gyro_thermal = np.zeros(3)
        self.base_imu_acc_bias_noise = 0.0001          # m/(sec^3*sqrt(Hz))
        self.base_imu_gyro_bias_noise = 0.0000309      # rad/(sec^2*sqrt(Hz))
        self.base_imu_acc_thermal_noise = 0.00001962   # m/(sec^2*sqrt(Hz))
        self.base_imu_gyro_thermal_noise = 0.000000873 # rad/(sec*sqrt(Hz))
    
    def reset(self, dt):
        self.gc = self.gc_init.copy()
        self.gv = self.gv_init.copy()
        self.base_imu_acc_bias = 0
        self.base_imu_gyro_bias = 0
        self.base_imu_acc_thermal = 0
        self.base_imu_gyro_thermal = 0
        self.robot.setState(self.gc, self.gv)
        self.update_states(dt)
        
    def update_states(self, dt):
        # get sim generalised joint data
        self.gc, self.gv = self.robot.getState()
        self.gf = self.robot.getGeneralizedForce()
        
        # update the base rotation of the robot in rotation matrix
        self.base_rotation = self._get_rotation_matrix_from_quaternion(self.gc[3:7])
        
        # find local base linear and angular velocity
        self.base_lin_vel = np.dot(self.base_rotation.transpose(), self.gv[0:3])
        self.base_ang_vel = np.dot(self.base_rotation.transpose(), self.gv[3:6])
        
        # Compute base acceleration numerically
        if self.base_lin_vel_prev is not None and self.base_ang_vel_prev is not None:
            self.base_lin_acc = (1.0/dt) * (self.base_lin_vel-self.base_lin_vel_prev)
            self.base_ang_acc = (1.0/dt) * (self.base_ang_vel-self.base_ang_vel_prev)
        self.base_lin_vel_prev = self.base_lin_vel.copy()
        self.base_ang_vel_prev = self.base_ang_vel.copy()
        
        """ Simulated IMU """
        # simulated imu data with noise
        self.imu_lin_acc = self.base_lin_acc - np.dot(self.base_rotation.transpose(), self.g)
        self.imu_ang_vel = self.base_ang_vel.copy()
        
        # Integrate IMU accelerometer/gyroscope bias terms forward
        self.base_imu_acc_bias += dt * (
            self.base_imu_acc_bias_noise/np.sqrt(dt)) * self.rng.standard_normal(3)
        self.base_imu_gyro_bias += dt * (
            self.base_imu_gyro_bias_noise/np.sqrt(dt)) * self.rng.standard_normal(3)
        
        # Add simulated IMU sensor thermal noise.
        self.base_imu_acc_thermal += (
            self.base_imu_acc_thermal_noise/np.sqrt(dt)) * self.rng.standard_normal(3)
        self.base_imu_gyro_thermal += (
            self.base_imu_gyro_thermal_noise/np.sqrt(dt)) * self.rng.standard_normal(3)
    
    def construct_pd_action(self, dt, kp, kd, des_pos, des_vel=None, des_trq=None):
        """
        Construct an action in the correct format for the PD controller.
        
        Args:
            dt (float): The control time step for the higher level controller.
            kp (ndarray): Array of kp gains.
            kd (ndarray): Array of kd gains.
            des_pos (ndarray): Array of desired joint positions.
            des_vel (ndarray): Array of desired joint velocities.
            des_trq (ndarray): Array of desired joint torques. 
        
        Returns:
            action (list): a list with the length: nJoint*5. For every 5 elements
                           of the list it goes (des_pos, kp, des_vel, kd, des_trq).
        """
        # Check if inputs are correct size
        if len(des_pos) != self.nJoints or len(kp) != self.nJoints or len(kd) != self.nJoints:
            raise ValueError(f"Check input array size, they should be of length {self.nJoints}")
        # if no desired velocity is input, generate it
        if des_vel is None:
            des_vel = (des_pos - self.get_joint_positions())/dt
        # if no desired torque is input, set to zero
        if des_trq is None:
            des_trq = np.zeros(self.nJoints)
        # construct action
        action_arrays = [des_pos, kp, des_vel, kd, des_trq]
        action = []
        for i in range(self.nJoints):
            for ii in range(len(action_arrays)):
                action.append(action_arrays[ii][i])
        return action
        
    def set_next_action(self, next_action, dt):
        if self.ctrl_mode == "simple_PD_control":
            des_vel = np.zeros(self.nJoints)
            des_trq = np.zeros(self.nJoints)
            des_pd_action = self.construct_pd_action(dt, self.jointKp, self.jointKd, next_action, des_vel, des_trq)
            des_trq = self.joint_controller.convert_to_torque(self.get_joint_positions(), self.get_joint_velocities(), des_pd_action)
            self.des_action[-self.nJoints:] = des_trq
        else:
            raise ValueError(f"Control mode {self.ctrl_mode} is not supported.")
    
    def apply_action(self):
        if self.ctrl_mode == "simple_PD_control":
            self.robot.setGeneralizedForce(self.des_action)
        else:
            raise ValueError(f"Control mode {self.ctrl_mode} is not supported.")
            
    def get_raw_base_imu_angular_velocity(self):
        return (self.imu_ang_vel + self.base_imu_gyro_bias + 
            self.base_imu_gyro_thermal + 0.015*self.rng.standard_normal(3))
    
    def get_base_imu_angular_velocity(self):
        return (self.imu_ang_vel + self.base_imu_gyro_bias + 
            self.base_imu_gyro_thermal)

    def get_raw_base_imu_linear_acceleration(self):
        return (self.imu_lin_acc + self.base_imu_acc_bias + 
            self.base_imu_acc_thermal + 0.015*self.rng.standard_normal(3))

    def get_base_imu_linear_acceleration(self):
        return (self.imu_lin_acc + self.base_imu_acc_bias + 
            self.base_imu_acc_thermal)

    def get_raw_joint_positions(self):
        return self.gc[-self.nJoints:] + 0.005*self.rng.standard_normal(len(self.gc[-self.nJoints:]))

    def get_joint_positions(self):
        return self.gc[-self.nJoints:]

    def get_raw_joint_velocities(self):
        return self.gv[-self.nJoints:] + 0.15*self.rng.standard_normal(len(self.gv[-self.nJoints:]))
        
    def get_joint_velocities(self):
        return self.gv[-self.nJoints:]

    def get_raw_joint_efforts(self):
        return self.gf[-self.nJoints:] + 1*self.rng.standard_normal(len(self.gf[-self.nJoints:]))
        
    def get_joint_efforts(self):
        return self.gf[-self.nJoints:]
        
    def get_base_position(self):
        return self.gc[:3]
        
    def get_base_quaternion(self):
        base_quaternion = self.gc[4:7].tolist()
        base_quaternion.append(self.gc[3])
        return np.array(base_quaternion)
        
    def get_base_euler(self):
        base_quaternion = self.gc[4:7].tolist()
        base_quaternion.append(self.gc[3])
        r = R.from_quat(base_quaternion)
        return r.as_euler('xyz')
    
    def get_base_pose_for_pin(self):
        base_pose = self.gc[:7]
        pin_base_pose = self.gc[:3].tolist()
        pin_base_pose.append(base_pose[4])
        pin_base_pose.append(base_pose[5])
        pin_base_pose.append(base_pose[6])
        pin_base_pose.append(base_pose[3])
        return np.array(pin_base_pose)
        
    def get_base_vel_for_pin(self):
        return self.gv[:6]
        
    def get_link_position(self, link_name):
        return self.robot.getFramePosition(link_name)
        
    def get_raw_limb_contact_forces(self, dt):
        self.contact_forces = np.zeros(len(self.limb_endeff_names))
        for contact in self.robot.getContacts():
            if (contact.skip()):
                continue
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[0]):
                self.contact_forces[0] = contact.getImpulse()[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[1]):
                self.contact_forces[1] = contact.getImpulse()[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[2]):
                self.contact_forces[2] = contact.getImpulse()[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[3]):
                self.contact_forces[3] = contact.getImpulse()[2]/dt
        return self.contact_forces + 1*self.rng.standard_normal(len(self.contact_forces))

    def get_limb_contact_forces(self, dt):
        self.contact_forces = np.zeros(len(self.limb_endeff_names))
        for contact in self.robot.getContacts():
            if (contact.skip()):
                continue
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[0]):
                self.contact_forces[0] = contact.getImpulse()[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[1]):
                self.contact_forces[1] = contact.getImpulse()[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[2]):
                self.contact_forces[2] = contact.getImpulse()[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[3]):
                self.contact_forces[3] = contact.getImpulse()[2]/dt
        return self.contact_forces

    def get_raw_limb_normal_contact_forces(self, dt):
        self.normal_contact_forces = np.zeros(len(self.limb_endeff_names))
        for contact in self.robot.getContacts():
            if (contact.skip()):
                continue
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[0]):
                self.normal_contact_forces[0] = np.dot(contact.getContactFrame().transpose(), contact.getImpulse())[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[1]):
                self.normal_contact_forces[1] = np.dot(contact.getContactFrame().transpose(), contact.getImpulse())[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[2]):
                self.normal_contact_forces[2] = np.dot(contact.getContactFrame().transpose(), contact.getImpulse())[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[3]):
                self.normal_contact_forces[3] = np.dot(contact.getContactFrame().transpose(), contact.getImpulse())[2]/dt
        return self.normal_contact_forces + self.rng.standard_normal(len(self.normal_contact_forces))

    def get_limb_normal_contact_forces(self, dt):
        self.normal_contact_forces = np.zeros(len(self.limb_endeff_names))
        for contact in self.robot.getContacts():
            if (contact.skip()):
                continue
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[0]):
                self.normal_contact_forces[0] = np.dot(contact.getContactFrame().transpose(), contact.getImpulse())[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[1]):
                self.normal_contact_forces[1] = np.dot(contact.getContactFrame().transpose(), contact.getImpulse())[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[2]):
                self.normal_contact_forces[2] = np.dot(contact.getContactFrame().transpose(), contact.getImpulse())[2]/dt
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[3]):
                self.normal_contact_forces[3] = np.dot(contact.getContactFrame().transpose(), contact.getImpulse())[2]/dt
        return self.normal_contact_forces

    def get_raw_limb_contact_forces_magnitude(self, dt):
        self.contact_forces_magnitude = np.zeros(len(self.limb_endeff_names))
        for contact in self.robot.getContacts():
            if (contact.skip()):
                continue
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[0]):
                self.contact_forces_magnitude[0] = LA.norm(contact.getImpulse()/dt)
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[1]):
                self.contact_forces_magnitude[1] = LA.norm(contact.getImpulse()/dt)
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[2]):
                self.contact_forces_magnitude[2] = LA.norm(contact.getImpulse()/dt)
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[3]):
                self.contact_forces_magnitude[3] = LA.norm(contact.getImpulse()/dt)
        return self.contact_forces_magnitude + self.rng.standard_normal(len(self.contact_forces_magnitude))

    def get_limb_contact_forces_magnitude(self, dt):
        self.contact_forces_magnitude = np.zeros(len(self.limb_endeff_names))
        for contact in self.robot.getContacts():
            if (contact.skip()):
                continue
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[0]):
                self.contact_forces_magnitude[0] = LA.norm(contact.getImpulse()/dt)
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[1]):
                self.contact_forces_magnitude[1] = LA.norm(contact.getImpulse()/dt)
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[2]):
                self.contact_forces_magnitude[2] = LA.norm(contact.getImpulse()/dt)
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[3]):
                self.contact_forces_magnitude[3] = LA.norm(contact.getImpulse()/dt)
        return self.contact_forces_magnitude

    def get_limb_contact_states(self):
        contact_states = np.zeros(len(self.limb_endeff_names))
        for contact in self.robot.getContacts():
            if (contact.skip()):
                continue
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[0]):
                contact_states[0] = 1
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[1]):
                contact_states[1] = 1
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[2]):
                contact_states[2] = 1
            if contact.getlocalBodyIndex() == self.robot.getBodyIdx(self.limb_endeff_names[3]):
                contact_states[3] = 1
        return contact_states
        
    def check_for_fail(self):
        failed = False
        for contact in self.robot.getContacts():
            if contact.getlocalBodyIndex() not in self.footIndx:
                failed = True
        return failed

    def get_base_rotation_matrix(self):
        return self.base_rotation

    def get_base_linear_velocity(self):
        return self.base_lin_vel
        
    def get_base_angular_velocity(self):
        return self.base_ang_vel
        
    def get_kinetic_energy(self):
        return self.robot.get_kinetic_energy()
    
    def get_potential_energy(self):
        return self.robot.getPotentialEnergy(np.array([0,0,-9.81]))

    def get_total_energy(self):
        return self.robot.getEnergy(np.array([0,0,-9.81]))
        
    def get_robot_mass(self):
        return self.robot.getTotalMass()
    
    def robot_object(self):
        return self.robot
    
    def nominal_gc(self, robot_id=0):
        return self.gc_init

    @property
    def nominal_gv(self, robot_id=0):
        return self.gv_init  
    
    @staticmethod
    def _get_rotation_matrix_from_quaternion(quaternion):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/

        # Extract the values from Q
        q0 = quaternion[0]
        q1 = quaternion[1]
        q2 = quaternion[2]
        q3 = quaternion[3]

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])

        return rot_matrix
        
        
        
