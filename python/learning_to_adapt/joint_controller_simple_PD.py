import numpy as np

# These values represent the indices of each field in the motor command tuple
POS_IDX = 0
KP_IDX = 1
VEL_IDX = 2
KD_IDX = 3
TRQ_IDX = 4

# Each motor command is a tuple (position, position_gain, velocity, velocity_gain. 
# torque)
ACT_DIM = 5

class JointController:
    """A simple motor model for legged robots."""
    def __init__(self, joint_names, torque_limits):
        """Initialize the joint controller.

        Args:
            joint_names (:obj:`list` of :obj:`str`): The joint names.
            torque_limits (ndarray): The joint torque limits.
        """
        self.joint_names = joint_names
        self.torque_limits = torque_limits

    def convert_to_torque(self, joint_pos, joint_vel, cmds):
        """Convert the commands (position control or torque control) to torque. 
        Each motor command is a tuple (position, position_gain, velocity, 
        velocity_gain, torque).

        Args:
            joint_pos (ndarray): Measured joint positions.
            joint_vel (ndarray): Measured joint velocities.
            cmds (ndarray): The tuple (q, kp, dq, kd, tau).

        Returns:
            applied_torque: The torque that needs to be applied to the motor.
        """
        assert len(cmds) == len(self.joint_names) * ACT_DIM
        applied_torques = []
        for idx, jn in enumerate(self.joint_names):
            applied_torques.append(
                cmds[idx*ACT_DIM+KP_IDX] * (
                    cmds[idx*ACT_DIM+POS_IDX] - joint_pos[idx]) + 
                cmds[idx*ACT_DIM+KD_IDX] * (
                    cmds[idx*ACT_DIM+VEL_IDX] - joint_vel[idx]) + 
                cmds[idx*ACT_DIM+TRQ_IDX]
            )
            applied_torques[idx] = np.clip(
                applied_torques[idx], 
                -1.0*self.torque_limits[idx], 
                self.torque_limits[idx]
            )
        #print(applied_torques)

        return applied_torques

