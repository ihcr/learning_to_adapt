class EngineInterface:
    def __init__(self, config, model_path_ground, model_path_robot):
        engine = str(config['engine'])

        # Set up a wrapper
        if engine == 'pybullet':
            from learning_to_adapt.pybullet import PyBulletWrapper
            self._wrapper = PyBulletWrapper(
                config, model_path_ground, model_path_robot)
        elif engine == 'raisim':
            from learning_to_adapt.raisim import RaiSimWrapper
            self._wrapper = RaiSimWrapper(
                config)
        else:
            raise ValueError(f'Engine {engine} is not Supported. Pick between pybullet or raisim.')

    def add_robot(self, cfg, model_path_robot, x, y):
        self._wrapper.add_robot(cfg, model_path_robot, x, y)

    def initialise_server(self):
        self._wrapper.initialise_server()

    def update_robot_states(self):
        self._wrapper.update_robot_states()

    def set_next_action(self, next_action, robot_id=0):
        self._wrapper.set_next_action(next_action, robot_id)

    def step(self, action, update_states=True, sleep=True):
        self._wrapper.step(action, update_states, sleep)

    def reset(self):
        self._wrapper.reset()

    def shutdown(self):
        self._wrapper.shutdown()

    def contruct_action(self, dt, kp, kd, des_pos, des_vel=None, des_trq=None):
        return self._wrapper.contruct_action(dt, kp, kd, des_pos, des_vel, des_trq)

    def remove_height_map(self):
        self._wrapper.remove_height_map()
        
    def create_static_box(self, xBox, yBox, zBox, xPos, yPos, zPos):
        self._wrapper.create_static_box(xBox, yBox, zBox, xPos, yPos, zPos)
        
    def create_new_height_map(self, freq, z_scale, y_size, x_size, y_samples, x_samples, octaves, lacunarity, gain, x_pos, y_pos):
        self._wrapper.create_new_height_map(freq, z_scale, y_size, x_size, y_samples, x_samples, octaves, lacunarity, gain, x_pos, y_pos)

    def create_tracking_sphere(self):
        self._wrapper.create_tracking_sphere()
        
    def update_tracking_sphere_pos(self, new_pos):
        self._wrapper.update_tracking_sphere_pos(new_pos)
    
    def create_tracking_camera(self):
        self._wrapper.create_tracking_camera()
        
    def fix_camera(self, cam_pos):
        self._wrapper.fix_camera(cam_pos)
        
    def free_camera(self):
        self._wrapper.free_camera()
    
    def get_time_since_start(self):
        return self._wrapper.get_time_since_start()
        
    def get_raw_base_imu_angular_velocity(self, robot_id=0):
        return self._wrapper.get_raw_base_imu_angular_velocity(robot_id)
        
    def get_base_imu_angular_velocity(self, robot_id=0):
        return self._wrapper.get_base_imu_angular_velocity(robot_id)
        
    def get_raw_base_imu_linear_acceleration(self, robot_id=0):
        return self._wrapper.get_raw_base_imu_linear_acceleration(robot_id)
        
    def get_base_imu_linear_acceleration(self, robot_id=0):
        return self._wrapper.get_base_imu_linear_acceleration(robot_id)
        
    def get_raw_joint_positions(self, robot_id=0):
        return self._wrapper.get_raw_joint_positions(robot_id)

    def get_joint_positions(self, robot_id=0):
        return self._wrapper.get_joint_positions(robot_id)
        
    def get_raw_joint_velocities(self, robot_id=0):
        return self._wrapper.get_raw_joint_velocities(robot_id)

    def get_joint_velocities(self, robot_id=0):
        return self._wrapper.get_joint_velocities(robot_id)

    def get_raw_joint_efforts(self, robot_id=0):
        return self._wrapper.get_raw_joint_efforts(robot_id)
        
    def get_joint_efforts(self, robot_id=0):
        return self._wrapper.get_joint_efforts(robot_id)

    def get_raw_limb_contact_forces(self, robot_id=0):
        return self._wrapper.get_raw_limb_contact_forces(robot_id)

    def get_limb_contact_forces(self, robot_id=0):
        return self._wrapper.get_limb_contact_forces(robot_id)
        
    def get_raw_limb_normal_contact_forces(self, robot_id=0):
        return self._wrapper.get_raw_limb_normal_contact_forces(robot_id)
       
    def get_limb_normal_contact_forces(self, robot_id=0):
        return self._wrapper.get_limb_normal_contact_forces(robot_id)
        
    def get_raw_limb_contact_forces_magnitude(self, robot_id=0):
        return self._wrapper.get_raw_limb_contact_forces_magnitude(robot_id)

    def get_limb_contact_forces_magnitude(self, robot_id=0):
        return self._wrapper.get_limb_contact_forces_magnitude(robot_id)
    
    def get_limb_contact_state(self, robot_id=0):
        return self._wrapper.get_limb_contact_state(robot_id)
    
    def check_for_fail(self, robot_id=0):
        return self._wrapper.check_for_fail(robot_id)
        
    def get_base_rotation_matrix(self, robot_id=0):
        return self._wrapper.get_base_rotation_matrix(robot_id)
        
    def get_base_linear_velocity(self, robot_id=0):
        return self._wrapper.get_base_linear_velocity(robot_id)
        
    def get_base_angular_velocity(self, robot_id=0):
        return self._wrapper.get_base_angular_velocity(robot_id)
        
    def get_limb_contact_states(self, robot_id=0):
        return self._wrapper.get_limb_contact_states(robot_id)
        
    def get_base_pose_for_pin(self, robot_id=0):
        return self._wrapper.get_base_pose_for_pin(robot_id)
        
    def get_base_vel_for_pin(self, robot_id=0):
        return self._wrapper.get_base_vel_for_pin(robot_id)
        
    def get_link_position(self, link_name, robot_id=0):
        return self._wrapper.get_link_position(link_name, robot_id)
        
    def get_base_position(self, robot_id=0):
        return self._wrapper.get_base_position(robot_id)
        
    def get_base_euler(self, robot_id=0):
        return self._wrapper.get_base_euler(robot_id)
        
    def get_base_quaternion(self, robot_id=0):
        return self._wrapper.get_base_quaternion(robot_id)
        
    def get_kinetic_energy(self, robot_id=0):
        return self._wrapper.get_kinetic_energy(robot_id)
        
    def get_potential_energy(self, robot_id=0):
        return self._wrapper.get_potential_energy(robot_id)

    def get_total_energy(self):
        return self._wrapper.get_total_energy(robot_id)
        
    def get_robot_mass(self, robot_id=0):
        return self._wrapper.get_robot_mass(robot_id)

    @property
    def nominal_gc(self):
        return self._wrapper.nominal_gc

    @property
    def nominal_gv(self):
        return self._wrapper.nominal_gv

    @property
    def generalized_coordinates(self):
        return self._wrapper.gc

    @property
    def generalized_velocities(self):
        return self._wrapper.gv

    @property
    def base_rotation(self):
        return self._wrapper.base_rotation
