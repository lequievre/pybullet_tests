"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr

forked from :

Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
This software may be modified and distributed under the terms of the
LGPL-2.1+ license.


FRANKA PANDA => num of joints = 12

i=0, name=panda_joint1, type=JOINT_REVOLUTE, lower=-2.9671, upper=2.9671, effort=87.0, velocity=2.175
i=1, name=panda_joint2, type=JOINT_REVOLUTE, lower=-1.8326, upper=1.8326, effort=87.0, velocity=2.175
i=2, name=panda_joint3, type=JOINT_REVOLUTE, lower=-2.9671, upper=2.9671, effort=87.0, velocity=2.175
i=3, name=panda_joint4, type=JOINT_REVOLUTE, lower=-3.1416, upper=0.0, effort=87.0, velocity=2.175
i=4, name=panda_joint5, type=JOINT_REVOLUTE, lower=-2.9671, upper=2.9671, effort=12.0, velocity=2.61
i=5, name=panda_joint6, type=JOINT_REVOLUTE, lower=-0.0873, upper=3.8223, effort=12.0, velocity=2.61
i=6, name=panda_joint7, type=JOINT_REVOLUTE, lower=-2.9671, upper=2.9671, effort=12.0, velocity=2.61
i=7, name=panda_joint8, type=JOINT FIXED, lower=0.0, upper=-1.0, effort=0.0, velocity=0.0
i=8, name=panda_hand_joint, type=JOINT FIXED, lower=0.0, upper=-1.0, effort=0.0, velocity=0.0
i=9, name=panda_finger_joint1, type=JOINT_PRISMATIC, lower=0.0, upper=0.04, effort=20.0, velocity=0.2
i=10, name=panda_finger_joint2, type=JOINT_PRISMATIC, lower=0.0, upper=0.04, effort=20.0, velocity=0.2
i=11, name=panda_grasptarget_hand, type=JOINT FIXED, lower=0.0, upper=-1.0, effort=0.0, velocity=0.0

<joint name="panda_grasptarget_hand" type="fixed">
            <parent link="panda_hand"/>
            <child link="panda_grasptarget"/>
            <origin rpy="0 0 0" xyz="0 0 0.07"/>
</joint>

limits :
('panda_joint1', 0)   -2.9671   2.9671
('panda_joint2', 1)   -1.8326   1.8326
('panda_joint3', 2)   -2.9671   2.9671
('panda_joint4', 3)   -3.1416   0.0
('panda_joint5', 4)   -2.9671   2.9671
('panda_joint6', 5)   -0.0873   3.8223
('panda_joint7', 6)   -2.9671   2.9671
('panda_finger_joint1', 9)   0.0   0.04
('panda_finger_joint2', 10)   0.0   0.04


"""

import pybullet as p
from gym.utils import seeding

import pybullet_data

import numpy as np
import math as m
from collections import OrderedDict

from statistics import mean

class PandaEnv:

    # dictionary than define initial joint position of the robot
    _joint_initial_position = OrderedDict([
        ('panda_joint1', 0.0), ('panda_joint2', -0.54), ('panda_joint3', 0.0),
        ('panda_joint4', -2.6), ('panda_joint5', -0.30), ('panda_joint6', 2.0),
        ('panda_joint7', 1.0), ('panda_finger_joint1', 0.02), ('panda_finger_joint2', 0.02)
   ])

    # dictionary than associate joint name to URDF file index
    _joint_name_to_index = OrderedDict([
         ('panda_joint1', 0), ('panda_joint2', 1), ('panda_joint3', 2),
         ('panda_joint4', 3), ('panda_joint5', 4), ('panda_joint6', 5),
         ('panda_joint7', 6), ('panda_finger_joint1', 9), ('panda_finger_joint2', 10)
    ])
    

    def __init__(self, physics_client_id, base_position=(0.0, 0, 0.625)):

        #print("Start PandaEnv  init !")

        self._physics_client_id = physics_client_id
        self._base_position = base_position # robot base position into the world when loaded URDF

        # Nb joint to move = 6 (panda_joint1.. panda_joint6)
        # We removed the panda_joint7 in order to prevent the rotation of the gripper
        self.joint_action_space = 6

        # workspace limit [[xmin,xmax], [ymin,ymax], [zmin,zmax]]
        self._workspace_lim = [[0.3, 0.65], [-0.3, 0.3], [0.65, 1.5]]

        # Cartesian rotation limit of end effector
        # Euler limit = [[rot_xmin,rot_xmax], [rot_ymin,rot_ymax], [rot_zmin,rot_zmax]]
        self._end_eff_euler_lim = [[-m.pi, m.pi], [-m.pi, m.pi], [-m.pi, m.pi]]  # euler limit

        # end effector index : <link name="panda_grasptarget">
        # index of link "panda_grasptarget" = 11
		# rmq : index of child link = index of joint
		# Example :
		# Child link of joint "panda_grasptarget_hand" = "panda_grasptarget"
		# index of joint "panda_grasptarget_hand" = 11, so index of link "panda_grasptarget" = 11
        self.end_eff_idx = 11

        # Pybullet id of robot franka panda loaded
        self.robot_id = None

        # init random seed
        self.seed()

        # load robot from URDF
        self.reset()
        
        

        #print("End PandaEnv  init !")

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
   
    def reset(self):

        # pybullet data path -> /home/laurent/test_rl/ve_pybullet/lib/python3.5/site-packages/pybullet_data
        pybullet_data_path = pybullet_data.getDataPath()
        #print("=> data path -> {0}".format(pybullet_data_path))

        # Add a search data path
        p.setAdditionalSearchPath(pybullet_data_path)
        # pybullet data path -> /home/laurent/test_rl/ve_pybullet/lib/python3.5/site-packages/pybullet_data
        # a subdirectory franka_panda contain the urdf of panda (panda.urdf).

        # Load robot model
        flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION
        self.robot_id = p.loadURDF("franka_panda/panda.urdf",
                                   basePosition=self._base_position, useFixedBase=True, flags=flags,
                                   physicsClientId=self._physics_client_id)

        assert self.robot_id is not None, "Failed to load the panda model"

        # reset joints to initial position
        for item in self._joint_name_to_index.items():
            joint_name = item[0]
            joint_index = item[1]

            #print("robot reset {} : {} = {}".format(joint_index,joint_name,self._joint_initial_position[joint_name]))

            p.resetJointState(self.robot_id, joint_index, self._joint_initial_position[joint_name], physicsClientId=self._physics_client_id)

            p.setJointMotorControl2(self.robot_id, joint_index, p.POSITION_CONTROL,
                                        targetPosition=self._joint_initial_position[joint_name],
                                        positionGain=0.2, velocityGain=1.0,
                                        physicsClientId=self._physics_client_id)

        # self.list_lower_limits -> lower limits of panda_joint(1..7)
        # self.list_upper_limits -> upper limits of panda_joint(1..7)
        # self.list_ranges -> joint ranges of panda_joint(1..7) = upper_limits - lower_limits
        # self.list_rest_pos -> resting state == initial_positions of panda_joint(1..7)
        self.list_lower_limits, self.list_upper_limits, self.list_ranges, self.list_rest_pos = self.get_joint_ranges()


    def get_workspace(self):
        return [i[:] for i in self._workspace_lim]

    def set_workspace(self, ws):
        self._workspace_lim = [i[:] for i in ws]

    def get_joint_ranges(self):
        list_lower_limits, list_upper_limits, list_joint_ranges, list_rest_poses = [], [], [], []

        for item in self._joint_name_to_index.items():
            joint_name = item[0]
            joint_index = item[1] 
            joint_info = p.getJointInfo(self.robot_id, joint_index, physicsClientId=self._physics_client_id)

            a_lower_limit, an_upper_limit = joint_info[8:10]
            #print(item, ' ', a_lower_limit, ' ' , an_upper_limit)
            
            a_range = an_upper_limit - a_lower_limit

            # For simplicity, assume resting state = initial position
            a_rest_pose = self._joint_initial_position[joint_name]

            list_lower_limits.append(a_lower_limit)
            list_upper_limits.append(an_upper_limit)
            list_joint_ranges.append(a_range)
            list_rest_poses.append(a_rest_pose)

        return list_lower_limits, list_upper_limits, list_joint_ranges, list_rest_poses

    def get_observation(self):
        # Create observation state
        observation = []
        observation_lim = []

        # Get state of the end-effector link
        state = p.getLinkState(self.robot_id, self.end_eff_idx, computeLinkVelocity=1,
                               computeForwardKinematics=1, physicsClientId=self._physics_client_id)

        # -------------------------------------- #
        # --- End Effector Cartesian 6D pose --- #
        # -------------------------------------- #
        pos = state[0] # Position of end effector
        orn = state[1] # Orientation of end effector

        observation.extend(list(pos))
        observation_lim.extend(list(self._workspace_lim))
        
        # cartesian orientation of end effector (euler angles)
        end_eff_euler = p.getEulerFromQuaternion(orn)
        observation.extend(list(end_eff_euler))  # euler roll, pitch, yaw
        observation_lim.extend(self._end_eff_euler_lim)  # euler limits
        
        # ------------------------ #
        # --- Joint positions  --- #
        # ------------------------ #

        list_joint_states = p.getJointStates(self.robot_id, self._joint_name_to_index.values(), physicsClientId=self._physics_client_id)

        # joint_state[0] -> The position value of this joint
        list_joint_positions = [joint_state[0] for joint_state in list_joint_states]

        observation.extend(list(list_joint_positions))

        observation_lim.extend([[self.list_lower_limits[i], self.list_upper_limits[i]] for i in range(0, len(self._joint_name_to_index.values()))])
        
        return observation, observation_lim

    def get_action_dim(self):
        return self.joint_action_space

    def get_observation_dim(self):
        obs, _ = self.get_observation()
        return len(obs)

    def apply_action(self, action):

        # --------------------- #
        # --- Joint control --- #
        # --------------------- #

        assert len(action) == self.joint_action_space, ('number of motor commands differs from number of motor to control', len(action))

        joint_idxs = tuple(self._joint_name_to_index.values())

        for i, val in enumerate(action):
            motor = joint_idxs[i]
            new_motor_pos = min(self.list_upper_limits[i], max(self.list_lower_limits[i], val))

            p.setJointMotorControl2(self.robot_id,
                                    motor,
                                    p.POSITION_CONTROL,
                                    targetPosition=new_motor_pos,
                                    positionGain=0.5, velocityGain=1.0,
                                    physicsClientId=self._physics_client_id)


    def debug_draw_eff_position(self):
		
		# index of panda_grasptarget link = 11
		# rmq : index of child link = index of joint
		# Example :
		# Child link of joint "panda_grasptarget_hand" = "panda_grasptarget"
		# index of joint "panda_grasptarget_hand" = 11, so index of link "panda_grasptarget" = 11
		
        panda_grasptarget_link = 11
        
        # Get state of the panda_grasptarget_hand link
        state_grasp = p.getLinkState(self.robot_id, panda_grasptarget_link, computeLinkVelocity=0,
                               computeForwardKinematics=1, physicsClientId=self._physics_client_id)

                 
        #p.addUserDebugLine(state_grasp[0], [state_grasp[0][0]+1.0, state_grasp[0][1], state_grasp[0][2]], lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
       
        # draw 3d landmarks of the base of robot (index = -1) and the end effector (index = self.end_eff_idx)
        size_line = 0.2
        p.addUserDebugLine([0, 0, 0], [size_line, 0, 0], [1, 0, 0], lineWidth=2.0, lifeTime=0, parentObjectUniqueId=self.robot_id,
                           parentLinkIndex=-1, physicsClientId=self._physics_client_id)
        p.addUserDebugLine([0, 0, 0], [0, size_line, 0], [0, 1, 0], lineWidth=2.0, lifeTime=0, parentObjectUniqueId=self.robot_id,
                           parentLinkIndex=-1, physicsClientId=self._physics_client_id)
        p.addUserDebugLine([0, 0, 0], [0, 0, size_line], [0, 0, 1], lineWidth=2.0, lifeTime=0, parentObjectUniqueId=self.robot_id,
                           parentLinkIndex=-1, physicsClientId=self._physics_client_id)

        pos_eff = state_grasp[0]
        
        p.addUserDebugLine(pos_eff, [pos_eff[0]+size_line,pos_eff[1],pos_eff[2]], lineColorRGB=[1, 0, 0], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
        p.addUserDebugLine(pos_eff, [pos_eff[0],pos_eff[1]+size_line,pos_eff[2]], lineColorRGB=[0, 1, 0], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
        p.addUserDebugLine(pos_eff, [pos_eff[0],pos_eff[1],pos_eff[2]+size_line], lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)

    def debug_gui(self):
		
		# draw eff position with an "*"
        self.debug_draw_eff_position()

        # workspace limit [[xmin,xmax], [ymin,ymax], [zmin,zmax]]
        # self._workspace_lim = [[0.3, 0.65], [-0.3, 0.3], [0.65, 1.5]]
        ws = self._workspace_lim
        p1 = [ws[0][0], ws[1][0], ws[2][0]]  # xmin,ymin,zmin
        p2 = [ws[0][1], ws[1][0], ws[2][0]]  # xmax,ymin,zmin
        p3 = [ws[0][1], ws[1][1], ws[2][0]]  # xmax,ymax,zmin
        p4 = [ws[0][0], ws[1][1], ws[2][0]]  # xmin,ymax,zmin

        # draw lines of the wokspace limit
        # add a 3d line specified by a 3d starting point (from) and end point (to), a color [red,green,blue]
        p.addUserDebugLine(p1, p2, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
        p.addUserDebugLine(p2, p3, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
        p.addUserDebugLine(p3, p4, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
        p.addUserDebugLine(p4, p1, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)

        p1_zmax = [ws[0][0], ws[1][0], ws[2][1]]  # xmin,ymin,zmax
        p2_zmax = [ws[0][1], ws[1][0], ws[2][1]]  # xmax,ymin,zmax
        p3_zmax = [ws[0][1], ws[1][1], ws[2][1]]  # xmax,ymax,zmax
        p4_zmax = [ws[0][0], ws[1][1], ws[2][1]]  # xmin,ymax,zmax

        p.addUserDebugLine(p1_zmax, p2_zmax, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
        p.addUserDebugLine(p2_zmax, p3_zmax, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
        p.addUserDebugLine(p3_zmax, p4_zmax, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
        p.addUserDebugLine(p4_zmax, p1_zmax, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)

        p.addUserDebugLine(p1, p1_zmax, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
        p.addUserDebugLine(p2, p2_zmax, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
        p.addUserDebugLine(p3, p3_zmax, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)
        p.addUserDebugLine(p4, p4_zmax, lineColorRGB=[0, 0, 1], lineWidth=2.0, lifeTime=0, physicsClientId=self._physics_client_id)

        
