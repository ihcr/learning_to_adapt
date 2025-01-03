import numpy as np
from numpy import linalg as LA
import raisimpy as raisim
import time
from scipy.spatial.transform import Rotation as R

from learning_to_adapt.joint_controller_simple_PD import JointController
from learning_to_adapt.robot import Robot

class RaiSimWrapper:
    def __init__(self, cfg):
        # set control mode- only PD and torque control available for now
        self._ctrl_mode = cfg['robot']['ctrl_mode']
        
        # set up world
        self._world = raisim.World()
        self.dt = cfg['time_steps']['simulation']
        self._world.setTimeStep(self.dt)
        self._world.addGround()
        
        # set up containers
        self._robots = []
        self._step_count = 0
        
        terr = raisim.TerrainProperties()
        terr.frequency = 0
        terr.zScale = 0.5
        terr.ySize = 10.0
        terr.xSize = 10.0
        terr.xSamples = 80
        terr.ySamples = 80
        terr.fractalOctaves = 8
        terr.fractalLacunarity = 2.0
        terr.fractalGain = 0.25
        terr.heightOffset = -(0.5/2)-0.0001
        self.hm = self._world.addHeightMap(0, 0, terr)
        self.hm.setAppearance("soil1")
        
        self.box_list = []
    
    def initialise_server(self):
        if len(self._robots) > 0:
            self._server = raisim.RaisimServer(self._world)
            self._server.launchServer(8080)
            self._server.focusOn(self._robots[0].robot_object())
            self._tracking_camera = False
            self._free_camera = False
        else:
            raise ValueError(f"No robot added, please use add_robot() first.")
    
    def add_robot(self, cfg, model_path_robot, x, y):
        robot_id = len(self._robots)
        joint_order = cfg['robot']['joint_order']
        self._fixed_base = cfg['robot']['fixed_base']
        robot = self._world.addArticulatedSystem(model_path_robot, joint_order=joint_order)
        self._robots.append(Robot(cfg, robot, robot_id, x, y))

    def update_robot_states(self):
        for i in range(len(self._robots)):
            self._robots[i].update_states(self.dt)
            
    def set_next_action(self, next_action, robot_id=0):
        self._robots[robot_id].set_next_action(next_action, self.dt)

    def step(self, des_action, update_states=True, sleep=True):
        if update_states:
            self.update_robot_states()
        for i in range(len(self._robots)):
            self._robots[i].apply_action()
        self._world.integrate()
        if sleep:
            self.sleep_for_time_step()
        if self._tracking_camera == True and self._free_camera == True:
            camera_pos = self.get_base_position()
            camera_pos[2] = self._robots[0].nominal_gc()[2] #self.gc_init[2]
            self.tracking_camera.setBodyType(raisim.BodyType.KINEMATIC)
            self.tracking_camera.setPosition(camera_pos)
            self.tracking_camera.setBodyType(raisim.BodyType.STATIC)
        self._step_count +=1

    def reset(self):
        self._step_count = 0
        for i in range(len(self._robots)):
            self._robots[i].reset(self.dt)

    def sleep_for_time_step(self):
        time.sleep(self._world.getTimeStep())
    
    # terrain management
    def remove_height_map(self):
        self._world.removeObject(self.hm)
    
    def create_new_height_map(self, freq, z_scale, y_size, x_size, y_samples, x_samples, octaves, lacunarity, gain, x_pos, y_pos):
        self.remove_height_map()
        terr = raisim.TerrainProperties()
        terr.frequency = freq
        terr.zScale = z_scale
        terr.ySize = y_size
        terr.xSize = x_size
        terr.xSamples = x_samples
        terr.ySamples = y_samples
        terr.fractalOctaves = octaves
        terr.fractalLacunarity = lacunarity
        terr.fractalGain = gain
        terr.heightOffset = -(z_scale/2)-0.0001
        self.hm = self._world.addHeightMap(x_pos, y_pos, terr)
        self.hm.setAppearance("0.7725,0.8431,0.7412,1")
        
    def create_static_box(self, xBox, yBox, zBox, xPos, yPos, zPos):
        self.box_list.append(self._world.addBox(xBox, yBox, zBox, 1.0))
        self.box_list[-1].setPosition(xPos, yPos, zPos)
        self.box_list[-1].setBodyType(raisim.BodyType.STATIC)
    
    def create_tracking_sphere(self):
        self.visual_sphere = self._server.addVisualSphere("tracking_sphere", 0.05, 1., 0., 0., 1.)
        
    def update_tracking_sphere_pos(self, new_pos):
        self.visual_sphere.setPosition(new_pos)
    
    def create_tracking_camera(self):
        self.tracking_camera = self._world.addSphere(1e-8,1e-8, collision_mask=0)
        #self.tracking_camera = self._server.addVisualSphere("tracking_sphere", 1e-8, 1., 0., 0., 1.)
        self._server.focusOn(self.tracking_camera)
        self.tracking_camera.setBodyType(raisim.BodyType.STATIC)
        self._tracking_camera = True
        self._free_camera = True
        
    def fix_camera(self, cam_pos):
        self.tracking_camera.setPosition(cam_pos)
        self.tracking_camera.setBodyType(raisim.BodyType.STATIC)
        self._free_camera = False
        
    def free_camera(self):
        self._free_camera = True
    
    def get_time_since_start(self):
        return self._step_count * self.dt
        
    def get_raw_base_imu_angular_velocity(self, robot_id=0):
        return self._robots[robot_id].get_raw_base_imu_angular_velocity()

    def get_base_imu_angular_velocity(self, robot_id=0):
        return self._robots[robot_id].get_base_imu_angular_velocity()

    def get_raw_base_imu_linear_acceleration(self, robot_id=0):
        return self._robots[robot_id].get_raw_base_imu_linear_acceleration()

    def get_base_imu_linear_acceleration(self, robot_id=0):
        return self._robots[robot_id].get_base_imu_linear_acceleration()

    def get_raw_joint_positions(self, robot_id=0):
        return self._robots[robot_id].get_raw_joint_positions()

    def get_joint_positions(self, robot_id=0):
        return self._robots[robot_id].get_joint_positions()

    def get_raw_joint_velocities(self, robot_id=0):
        return self._robots[robot_id].get_raw_joint_velocities()
        
    def get_joint_velocities(self, robot_id=0):
        return self._robots[robot_id].get_joint_velocities()

    def get_raw_joint_efforts(self, robot_id=0):
        return self._robots[robot_id].get_raw_joint_efforts()
        
    def get_joint_efforts(self, robot_id=0):
        return self._robots[robot_id].get_joint_efforts()
        
    def get_base_position(self, robot_id=0):
        return self._robots[robot_id].get_base_position()
        
    def get_base_quaternion(self, robot_id=0):
        return self._robots[robot_id].get_base_quaternion()
        
    def get_base_euler(self, robot_id=0):
        return self._robots[robot_id].get_base_euler()
    
    def get_base_pose_for_pin(self, robot_id=0):
        return self._robots[robot_id].get_base_pose_for_pin()
        
    def get_base_vel_for_pin(self, robot_id=0):
        return self._robots[robot_id].get_base_vel_for_pin()
        
    def get_link_position(self, link_name, robot_id=0):
        return self._robots[robot_id].get_link_position(link_name)
        
    def get_raw_limb_contact_forces(self, robot_id=0):
        return self._robots[robot_id].get_raw_limb_contact_forces(self.dt)

    def get_limb_contact_forces(self, robot_id=0):
        return self._robots[robot_id].get_limb_contact_forces(self.dt)

    def get_raw_limb_normal_contact_forces(self, robot_id=0):
        return self._robots[robot_id].get_raw_limb_normal_contact_forces(self.dt)

    def get_limb_normal_contact_forces(self, robot_id=0):
        return self._robots[robot_id].get_limb_normal_contact_forces(self.dt)

    def get_raw_limb_contact_forces_magnitude(self, robot_id=0):
        return self._robots[robot_id].get_raw_limb_contact_forces_magnitude(self.dt)

    def get_limb_contact_forces_magnitude(self, robot_id=0):
        return self._robots[robot_id].get_limb_contact_forces_magnitude(self.dt)

    def get_limb_contact_states(self, robot_id=0):
        return self._robots[robot_id].get_limb_contact_states()
        
    def check_for_fail(self, robot_id=0):
        return self._robots[robot_id].get_limb_contact_states()

    def get_base_rotation_matrix(self, robot_id=0):
        return self._robots[robot_id].get_base_rotation_matrix()

    def get_base_linear_velocity(self, robot_id=0):
        return self._robots[robot_id].get_base_linear_velocity()
        
    def get_base_angular_velocity(self, robot_id=0):
        return self._robots[robot_id].get_base_angular_velocity()
        
    def get_kinetic_energy(self, robot_id=0):
        return self._robots[robot_id].get_kinetic_energy()
    
    def get_potential_energy(self, robot_id=0):
        return self._robots[robot_id].get_potential_energy()

    def get_total_energy(self, robot_id=0):
        return self._robots[robot_id].get_total_energy()
        
    def get_robot_mass(self, robot_id=0):
        return self._robots[robot_id].get_robot_mass()

    def shutdown(self):
        self._server.killServer()
        
    @property
    def nominal_gc(self, robot_id=0):
        return self._robots[robot_id].nominal_gc()

    @property
    def nominal_gv(self, robot_id=0):
        return self._robots[robot_id].nominal_gv()
       
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
