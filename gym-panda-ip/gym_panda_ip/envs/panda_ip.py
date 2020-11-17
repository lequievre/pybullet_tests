"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

import pybullet as p
import pybullet_data


class pandaIP:

    # Init robot franka positions
    # Set initial positions
    initial_positions = {
        'panda_joint1': 0.0, 'panda_joint2': -0.54, 'panda_joint3': 0.0,
        'panda_joint4': -2.6, 'panda_joint5': -0.30, 'panda_joint6': 2.0,
        'panda_joint7': 1.0, 'panda_finger_joint1': 0.02, 'panda_finger_joint2': 0.02,
        }

    def __init__(self, physicsClientId, base_position=(0.0, 0, 0.625)):
        self._physics_client_id = physicsClientId
        self._base_position = base_position
        
        self._num_dof = 7
        self._joint_name_to_ids = {}
        
        self.end_eff_idx = 11  # index=11 in joint_info array -> name=panda_grasptarget_hand, type=JOINT FIXED
        
        self.robot_id = None
        
        self.reset()
    
    def reset(self):
        flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION
        self.robot_id = p.loadURDF("franka_panda/panda.urdf",
            basePosition=self._base_position, 
            useFixedBase=True, 
            flags=flags, 
            physicsClientId=self._physics_client_id)
        assert self.robot_id is not None, "Failed to load the panda URDF model !"

         # reset joints to home position
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self._physics_client_id)
        idx = 0
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i, physicsClientId=self._physics_client_id)
            joint_name = joint_info[1].decode("UTF-8")
            joint_type = joint_info[2]

            if joint_type is p.JOINT_REVOLUTE or joint_type is p.JOINT_PRISMATIC:
                assert joint_name in self.initial_positions.keys()

                self._joint_name_to_ids[joint_name] = i

                p.resetJointState(self.robot_id, i, self.initial_positions[joint_name], physicsClientId=self._physics_client_id)
                p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL,
                                        targetPosition=self.initial_positions[joint_name],
                                        positionGain=0.2, velocityGain=1.0,
                                        physicsClientId=self._physics_client_id)

                idx += 1
