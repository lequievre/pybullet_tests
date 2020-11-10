#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

import pybullet as p
import time
import pybullet_data

# Can alternatively pass in p.DIRECT for for non-graphical version
physicsClient = p.connect(p.GUI)

# Set Gravity to the environment
p.setGravity(0, 0, -10, physicsClientId=physicsClient)

# Get the default pybullet data path
a_data_path = pybullet_data.getDataPath()
print("=> data path -> {0}".format(a_data_path))

# Add a search data path
p.setAdditionalSearchPath(a_data_path)

# data path -> /home/laurent/.local/lib/python3.8/site-packages/pybullet_data
# a subdirectory franka_panda contain the urdf of panda (panda.urdf).

# load a franka panda arm robot at a position of [0, 0, 0]
robotId = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0])

# print joint infos
num_joints = p.getNumJoints(robotId, physicsClientId=physicsClient)
print("=> num of joints = {0}".format(num_joints))


# Set robot franka to initial positions
# Set initial positions
initial_positions = {
        'panda_joint1': 0.0, 'panda_joint2': -0.54, 'panda_joint3': 0.0,
        'panda_joint4': -2.6, 'panda_joint5': -0.30, 'panda_joint6': 2.0,
        'panda_joint7': 1.0, 'panda_finger_joint1': 0.02, 'panda_finger_joint2': 0.02,
    }
    
idx = 0
joint_name_to_ids = {}
joint_type_name = ""

switcher_type_name = {
        p.JOINT_REVOLUTE: "JOINT_REVOLUTE",
        p.JOINT_PRISMATIC: "JOINT_PRISMATIC"
    }

for i in range(num_joints):
  joint_info = p.getJointInfo(robotId, i, physicsClientId=physicsClient)
  joint_name = joint_info[1].decode("UTF-8")
  joint_type = joint_info[2]
  #print("joint n={0}, name -> {1}, type -> {2}".format(i,joint_name, joint_type))
  
  if joint_type is p.JOINT_REVOLUTE or joint_type is p.JOINT_PRISMATIC:
      assert joint_name in initial_positions.keys()
     
      joint_type_name = switcher_type_name.get(joint_type,"Invalid type")
                    
      print("joint {0}, type:{1} -> {2}".format(joint_name,joint_type,joint_type_name))
      joint_name_to_ids[joint_name] = i

      p.resetJointState(robotId, i, initial_positions[joint_name], physicsClientId=physicsClient)
      p.setJointMotorControl2(robotId, i, p.POSITION_CONTROL,
                                        targetPosition=initial_positions[joint_name],
                                        positionGain=0.2, velocityGain=1.0,
                                        physicsClientId=physicsClient)

      idx += 1

# 12 joints in total, 9 joints used
print("=> Num total of joints={0}, num of joints with inital position={1}".format(num_joints,idx))

# joint name used ->{'panda_joint1': 0, 'panda_joint2': 1, 'panda_joint3': 2, 'panda_joint4': 3, 'panda_joint5': 4, 'panda_joint6': 5, 'panda_joint7': 6, 'panda_finger_joint1': 9, 'panda_finger_joint2': 10}
print("joint name used ->{0}".format(joint_name_to_ids))

# Press enter to continue
try:
    input("Press enter to disconnect")
except SyntaxError:
    pass
    
# disconnect from the bullet environment
p.disconnect()
