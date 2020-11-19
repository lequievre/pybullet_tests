"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

import os
import pybullet as p
import pybullet_data

import numpy as np
import math

"""import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.sys.path.insert(0, currentdir)"""


class PandaIP:
    
    # Init robot franka positions
    # Set initial positions
    initial_positions = {
        'panda_joint1': 0.0, 'panda_joint2': -0.215, 'panda_joint3': 0.0,
        'panda_joint4': -2.57, 'panda_joint5': 0.0, 'panda_joint6': 2.356,
        'panda_joint7': 2.356, 'panda_finger_joint1': 0.08, 'panda_finger_joint2': 0.08,
        }
    
    

    def __init__(self, physicsClientId=None, urdfRoot=pybullet_data.getDataPath(), base_position=(0.0, 0.0, 0.0)):
        self._physics_client_id = physicsClientId
        self._urdfRoot = urdfRoot
        self._base_position = base_position
        
        self._num_dof = 7
        self._joint_name_to_ids = {}
        
        self.end_eff_idx = 11  # index=11 in joint_info array -> name=panda_grasptarget_hand, type=JOINT FIXED
        
        self.robot_id = None
        
        # Add a search data path
        p.setAdditionalSearchPath(self._urdfRoot)
         
        self.reset()
    
    def reset(self):
        flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION
        
        
        self.robot_id = p.loadURDF(os.path.join(self._urdfRoot, "franka_panda/panda.urdf"),
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
                
    def get_observation(self):
        # get position of end effector link (index = 11)
        state_robot = p.getLinkState(self.robot_id, self.end_eff_idx)[0]
        
        # get joint value of 2 fingers joints (index = 9 and 10)
        index_finger_joint1 = self._joint_name_to_ids["panda_finger_joint1"]
        index_finger_joint2 = self._joint_name_to_ids["panda_finger_joint2"]
        state_fingers = (p.getJointState(self.robot_id,index_finger_joint1)[0], p.getJointState(self.robot_id, index_finger_joint2)[0])
        
        self.observation = state_robot + state_fingers
        
        return np.array(self.observation).astype(np.float32)
    
    def apply_action(self, action):
        
        orientation = p.getQuaternionFromEuler([0.,-math.pi,math.pi/2.])
        
        dv = 0.005
        dx = action[0] * dv
        dy = action[1] * dv
        dz = action[2] * dv
        fingers = action[3]
        
        currentPose = p.getLinkState(self.robot_id, self.end_eff_idx)
        currentPosition = currentPose[0]
        newPosition = [currentPosition[0] + dx,
                       currentPosition[1] + dy,
                       currentPosition[2] + dz]
        
        jointPoses = p.calculateInverseKinematics(self.robot_id,self.end_eff_idx,newPosition, orientation)[0:7]

        p.setJointMotorControlArray(self.robot_id, list(range(7))+[9,10], p.POSITION_CONTROL, list(jointPoses)+2*[fingers])

        
    def get_state_robot_effector(self):
       return p.getLinkState(self.robot_id, self.end_eff_idx)[0]
   
    def get_state_fingers(self):
        index_finger_joint1 = self._joint_name_to_ids["panda_finger_joint1"]
        index_finger_joint2 = self._joint_name_to_ids["panda_finger_joint2"]
        return (p.getJointState(self.robot_id,index_finger_joint1)[0], p.getJointState(self.robot_id, index_finger_joint2)[0])
   