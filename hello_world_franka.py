#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

# lib pybullet -> simulation and learning environment
# pip3 install pybullet --upgrade --user
import pybullet as p
import pybullet_data

import math


switcher_type_name = {
        p.JOINT_REVOLUTE: "JOINT_REVOLUTE",
        p.JOINT_PRISMATIC: "JOINT_PRISMATIC",
        p.JOINT_SPHERICAL: "JOINT SPHERICAL",
        p.JOINT_PLANAR: "JOINT PLANAR",
        p.JOINT_FIXED: "JOINT FIXED"
    }

def printAllInfo(robotid, clientId):
  print("=================================")
  print("All Robot joints info")
  num_joints = p.getNumJoints(robotid, physicsClientId=clientId)
  print("=> num of joints = {0}".format(num_joints))
  for i in range(num_joints):
    joint_info = p.getJointInfo(robotid, i, physicsClientId=clientId)
    joint_name = joint_info[1].decode("UTF-8")
    joint_type = joint_info[2]
    joint_type_name = switcher_type_name.get(joint_type,"Invalid type")
    joint_lower_limit, joint_upper_limit = joint_info[8:10]
    print("i={0}, name={1}, type={2}, lower={3}, upper={4}".format(i,joint_name,joint_type_name,joint_lower_limit,joint_upper_limit))
  print("=================================")
  

# Can alternatively pass in p.DIRECT for for non-graphical version
physicsClient = p.connect(p.GUI)

# enable keyboard events
p.configureDebugVisualizer( p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1 )

# Set Gravity to the environment
p.setGravity(0, 0, -10, physicsClientId=physicsClient)

# Get the default pybullet data path
a_data_path = pybullet_data.getDataPath()
print("=> data path -> {0}".format(a_data_path))

# Add a search data path
p.setAdditionalSearchPath(a_data_path)

# data path -> /home/laurent/.local/lib/python3.8/site-packages/pybullet_data
# a subdirectory franka_panda contain the urdf of panda (panda.urdf).

# load a franka panda arm robot at a default position of [0, 0, 0]
flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | p.URDF_USE_INERTIA_FROM_FILE | p.URDF_USE_SELF_COLLISION
robotId = p.loadURDF("franka_panda/panda.urdf", basePosition=[0.0,0.0,0.0], useFixedBase=True,flags=flags)
assert robotId is not None, "Failed to load the panda model"

# load a table, a tray, an object to grasp
tableUid = p.loadURDF("table/table.urdf", basePosition=[0.5,0,-0.65])
assert tableUid is not None, "Failed to load the table model"

trayUid = p.loadURDF("tray/traybox.urdf", basePosition=[0.65,0,0])
assert trayUid is not None, "Failed to load the traybox model"

objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7,0,0.1])
assert objectUid is not None, "Failed to load the object model"

printAllInfo(robotId,physicsClient)

# reset the initial view of the environment (to be closer to robot)
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])


# print joint infos
num_joints = p.getNumJoints(robotId, physicsClientId=physicsClient)
print("=> num of joints = {0}".format(num_joints))

# Init robot franka positions
# Set initial positions
initial_positions = {
        'panda_joint1': 0.0, 'panda_joint2': -0.54, 'panda_joint3': 0.0,
        'panda_joint4': -2.6, 'panda_joint5': -0.30, 'panda_joint6': 2.0,
        'panda_joint7': 1.0, 'panda_finger_joint1': 0.02, 'panda_finger_joint2': 0.02,
    }
   
# Set grasp positions 
pose_grasp_positions = {
        'panda_joint1': 0.0, 'panda_joint2': math.pi/4., 'panda_joint3': 0.0,
        'panda_joint4': -math.pi/2., 'panda_joint5': 0.0, 'panda_joint6': 3*math.pi/4,
        'panda_joint7': -math.pi/4., 'panda_finger_joint1': 0.08, 'panda_finger_joint2': 0.08,
    }   
    
idx = 0
joint_name_to_ids = {}
joint_type_name = ""


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

      # You can reset the state of the joint. It is best only to do this at the start, while not running the
      # simulation: resetJointState overrides all physics simulation.
      p.resetJointState(robotId, i, initial_positions[joint_name], physicsClientId=physicsClient)
      
      # We can control a robot by setting a desired control mode for one or more joint motors. During
      # the stepSimulation the physics engine will simulate the motors to reach the given target value
      # that can be reached within the maximum motor forces and other constraints. Each revolute joint
      # and prismatic joint is motorized by default. There are 3 different motor control modes: position
      # control, velocity control and torque control.
      p.setJointMotorControl2(robotId, i, p.POSITION_CONTROL,
                                        targetPosition=initial_positions[joint_name],
                                        positionGain=0.2, velocityGain=1.0,
                                        physicsClientId=physicsClient)

      idx += 1

# 12 joints in total, 9 joints used
print("=> Num total of joints={0}, num of joints with inital position={1}".format(num_joints,idx))

# joint name used ->{'panda_joint1': 0, 'panda_joint2': 1, 'panda_joint3': 2, 'panda_joint4': 3, 'panda_joint5': 4, 'panda_joint6': 5, 'panda_joint7': 6, 'panda_finger_joint1': 9, 'panda_finger_joint2': 10}
print("joint name used ->{0}".format(joint_name_to_ids))


while True:
    
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, physicsClientId=physicsClient)
    
    keys = p.getKeyboardEvents()
    
    # Enter event = 65309
    if 65309 in keys:
      break
  

    if 112 in keys: # Go to grasp position (with letter 'p' = 112)
      for joint_name in joint_name_to_ids.keys():
          #print("joint : {0}, id : {1}, pose: {2}".format(joint_name,joint_name_to_ids[joint_name],pose_grasp_positions[joint_name]))
          p.setJointMotorControl2(robotId, joint_name_to_ids[joint_name], p.POSITION_CONTROL,
                    targetPosition=pose_grasp_positions[joint_name],
                    positionGain=0.2, velocityGain=1.0,
                    physicsClientId=physicsClient)
                    
    elif 105 in keys: # Go to initial position (with letter 'i' = 105)
       for joint_name in joint_name_to_ids.keys():
           p.setJointMotorControl2(robotId, joint_name_to_ids[joint_name], p.POSITION_CONTROL,
                    targetPosition=initial_positions[joint_name],
                    positionGain=0.2, velocityGain=1.0,
                    physicsClientId=physicsClient)
                    
    elif 100 in keys: # Descends vertically on the object (with letter 'd' = 100)
        p.setJointMotorControl2(robotId, 1, 
                                p.POSITION_CONTROL,targetPosition=math.pi/4.+.15,
                                positionGain=0.2, velocityGain=1.0,
                                physicsClientId=physicsClient)
        p.setJointMotorControl2(robotId, 3, 
                                p.POSITION_CONTROL,targetPosition=-math.pi/2.+.15,
                                positionGain=0.2, velocityGain=1.0,
                                physicsClientId=physicsClient)
                    
    elif 103 in keys: # Grasp the object by using force (with letter 'g' = 103)
        p.setJointMotorControl2(robotId, 9, 
                                p.POSITION_CONTROL,targetPosition=0.0, force = 300,
                                physicsClientId=physicsClient)
        p.setJointMotorControl2(robotId, 10, 
                                p.POSITION_CONTROL,targetPosition=0.0, force = 300,
                                physicsClientId=physicsClient)
    
    elif 114 in keys: # Remove the object from the traybox (with letter 'r' = 114)
        p.setJointMotorControl2(robotId, 1, 
                                p.POSITION_CONTROL,targetPosition=math.pi/4.-1,
                    physicsClientId=physicsClient)
        p.setJointMotorControl2(robotId, 3, 
                                p.POSITION_CONTROL,targetPosition=-math.pi/2.-1,
                    physicsClientId=physicsClient)
       
    	   
    p.stepSimulation(physicsClientId=physicsClient)
    
   
# disconnect from the bullet environment
p.disconnect()
