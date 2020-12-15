__author__ = "Laurent LEQUIEVRE"
__copyright__ = "Copyright 2020, Institut Pascal UMR6602"
__credits__ = ["Laurent LEQUIEVRE Research Engineer, CNRS (France)"]
__license__ = "GNU GPLv3"
__version__ = "1.0.0"
__maintainer__ = "Laurent LEQUIEVRE"
__email__ = "laurent.lequievre@uca.fr"
__status__ = "Development"

"""
Links and Joints relations (KUKA iiwa robot with a wsg50 gripper)  :
-----------------------------------------------------------------

lbr_iiwa_link_0 - J0 (revolute) - lbr_iiwa_link_1 - J1 (revolute) - lbr_iiwa_link_2 - J2 (revolute) - lbr_iiwa_link_3 - J3 (revolute) - lbr_iiwa_link_4 - J4 (revolute) - lbr_iiwa_link_5 - J5 (revolute) - lbr_iiwa_link_6 - J6 (revolute) - lbr_iiwa_link_7 - 

lbr_iiwa_link_7 - gripper_to_arm (continuous) - base_link - base_left_finger_joint (revolute) - left_finger - left_finger_base_joint (fixed) - left_finger_base - left_base_tip_joint (revolute) - left_finger_tip

                                                base_link - base_right_finger_joint (revolute) - right_finger - right_finger_base_joint (fixed) - right_finger_base - right_base_tip_joint (revolute) - right_finger_tip
"""

import pybullet as p
import os, inspect

# Get and print infos about current and parent directory
currentframe = inspect.currentframe()
print("current file name={0}".format(inspect.getfile(inspect.currentframe())))
print("abs path of file={0}".format(os.path.abspath(inspect.getfile(inspect.currentframe()))))
print("current dir of file={0}".format(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))))

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

print("parent dir of file={0}".format(parentdir))

physicsClient = p.connect(p.GUI)

# Load SDF file of Kuka robot
objects = p.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")
kukaUid = objects[0]

# Set Gravity to the environment
#p.setGravity(0, 0, -9.81, physicsClientId=physicsClient)

# Get number of joints of kuka robot
numJoints = p.getNumJoints(kukaUid)
print("num joints of kuka robot={0}".format(numJoints))

# Get informations about joints and parent/child links
for i in range(numJoints):
	jointInfo = p.getJointInfo(kukaUid, i) # get info about Joint indexed by i
	ll, ul = jointInfo[8:10] # get lower and upper joint limit
	jr = ul - ll # calculate joint range (upper - limit)
	qIndex = jointInfo[3] # qIndex=-1 when joint is fixed
	
	jointName=str(jointInfo[1].decode("utf-8")) # get joint name
	childLinkName=str(jointInfo[12].decode("utf-8")) # get child link name of the joint
	
	parentIndex=jointInfo[16] # get index of parent link of the joint
	# Take Care : In bullet linkIndex == jointIndex
	# So 'parentIndex' is the parent link index of the joint and also the index of the joint associated to that link
	
	if (parentIndex!=-1):
		jointInfoParent=p.getJointInfo(kukaUid, parentIndex) # get joint info with of the parent link index 
		parentLinkName=str(jointInfoParent[12].decode("utf-8")) # get the child link name of that joint, this is the name of the parent link of the original joint
	else:
		parentLinkName="None"

	# I don't understand the utility of qIndex value ???
	# qIndex=-1 when joint is fixed, it is the case of 'left_finger_base_joint' and 'right_finger_base_joint'
	if qIndex > -1:
		print("i={0}, joint name=\"{1}\" (qIndex={2})".format(i,jointName,qIndex))
		print("-> link child=\"{0}\", parent=\"{1}\"".format(childLinkName,parentLinkName))
		print("-> limits lower={0}, upper={1}, range={2}".format(ll,ul,jr))
	else:
		print("** i={0}, joint name=\"{1}\" (qIndex={2})".format(i,jointName,qIndex))
		print("-> link child=\"{0}\", parent=\"{1}\"".format(childLinkName,parentLinkName))
		print("-> limits lower={0}, upper={1}, range={2}".format(ll,ul,jr))


# Get infos on end effector and gripper link
# end effector link = 'lbr_iiwa_link_7', gripper link = 'base_link'
kukaEndEffectorIndex = 6
kukaGripperIndex = 7

jointInfoEndEffector=p.getJointInfo(kukaUid, kukaEndEffectorIndex)
linkNameEndEffector=str(jointInfoEndEffector[12].decode("utf-8"))
jointInfoGripper=p.getJointInfo(kukaUid, kukaGripperIndex)
linkNameGripper=str(jointInfoGripper[12].decode("utf-8"))

print("-> link : end effector={0}, gripper={1}".format(linkNameEndEffector,linkNameGripper))

# reset the initial view of the environment (to be closer to robot)
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.35,0.0,0.7])

# reset the position and orientation of the base (root) of the robot kuka
p.resetBasePositionAndOrientation(kukaUid, [-0.100000, 0.000000, 0.070000],
                                      [0.000000, 0.000000, 0.000000, 1.000000])
    
#Add sliders for all joints (useful to move the joints)

joint_name_to_ids = {} # dictionary to associate a joint name to index
joint_name_to_slider = {} # dictionary to assciate a joint name to a slider

# Put kuka robot in an 'initial position' for grasping
initial_positions = [0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539]
joint_lower  = []
joint_upper = []
joint_range = []
joint_index = 0
for i in range(numJoints):
	jointInfo = p.getJointInfo(kukaUid, i) # get joint info of joint indexed by i
	jointName=str(jointInfo[1].decode("utf-8")) # get name of the joint
	jointType = jointInfo[2] # get type of the joint (only revolute joints will have a slider)
	
	if (jointType == p.JOINT_REVOLUTE):
		ll, ul = jointInfo[8:10] # get lower and upper limits
		
		if (joint_index < 7): # only for the 7 first joints
			joint_value = initial_positions[joint_index]
			joint_lower.append(ll)
			joint_upper.append(ul)
			joint_range.append(ul-ll)
			p.setJointMotorControl2(kukaUid,joint_index,p.POSITION_CONTROL,targetPosition=joint_value,positionGain=0.2,velocityGain=1.0,physicsClientId=physicsClient)
		else:
			joint_value = 0.0
		
		slider = p.addUserDebugParameter(jointName, ll, ul, joint_value) # add a slider for that joint with the limits
		joint_name_to_ids[jointName] = i # save the index of the joint name
		joint_name_to_slider[jointName] = slider  # save the slider of the joint name
		joint_index=joint_index+1
		
	p.stepSimulation(physicsClientId=physicsClient) # let time to update the GUI
	
#maxForce = 200 # define a maximum force value for moving joints with the sliders
print("limits -> lower={0}, upper={1}, range={2}".format(joint_lower,joint_upper,joint_range))


# set some useful parameters necessary when printing position and orientation of end effector link
textColor = [0.0, 0.0, 0.0]
shift = 0.05
str_pos = ""

# Print position and orientation of kuka end effector link.
# textPosition is relative to position of end effector link (parentLinkIndex=kukaEndEffectorIndex)
idEEText = p.addUserDebugText(str_pos, [shift, 0, .1],
				   textColor, textSize=15.0,
				   parentObjectUniqueId=kukaUid,
				   parentLinkIndex=kukaEndEffectorIndex)


# Print joint values after an IK of EEF link position and orientation (to debug IK)
str_ik_pos = ""
idEEIKText = p.addUserDebugText(str_ik_pos, [-1.0, -0.5, 0.0],
				   textColor, textSize=2.0,
				   parentObjectUniqueId=kukaUid)

#jointPoses = p.calculateInverseKinematics(self.kukaUid, self.kukaEndEffectorIndex, pos,
#                                                    orn, self.ll, self.ul, self.jr, self.rp)
while True:
	# look if there is keyboard event and put it in a dictionary named keys
	keys = p.getKeyboardEvents()

	# 'Enter' event = 65309, if so .. break the loop
	if 65309 in keys:
	  break
	
	# Read the slider value of each joint name
	for name in joint_name_to_slider.keys():
		slider = joint_name_to_slider[name] # get the slider of that joint name
		index = joint_name_to_ids[name] # get the index of that joint name
		
		value = p.readUserDebugParameter(slider) # read the slider value
		
		# apply that value to the joint (in Position)
		p.setJointMotorControl2(kukaUid,index,p.POSITION_CONTROL,targetPosition=value,positionGain=0.2,velocityGain=1.0,physicsClientId=physicsClient)

	
	state = p.getLinkState(kukaUid, kukaEndEffectorIndex)
	pos = state[4] # get position of center of mass in world cartesian position
	orn = state[5] # get orientation of center of mass (in quaternion)
	euler = p.getEulerFromQuaternion(orn) # convert quaternion to euler angles
	
	str_pos = "(" + str(round(pos[0],2)) + "," + str(round(pos[1],2)) + "," + str(round(pos[2],2)) + "), (" + str(round(euler[0],2)) + "," + str(round(euler[1],2)) + "," + str(round(euler[2],2)) + ")"

	p.removeUserDebugItem(idEEText)

	# Print position and orientation of kuka end effector link.
	# textPosition is relative to position of end effector link (parentLinkIndex=kukaEndEffectorIndex)
	idEEText = p.addUserDebugText(text=str_pos, textPosition=[shift, 0, .1],
				   textColorRGB=textColor,
				   parentObjectUniqueId=kukaUid,
				   parentLinkIndex=kukaEndEffectorIndex)
		
	# calculate IK of EEF to get joint values
	jointPoses = p.calculateInverseKinematics(kukaUid, kukaEndEffectorIndex, pos,
                                                    orn, joint_lower, joint_upper, joint_range, initial_positions)
				   
	
	# Print as debug the joint values
	str_ik_pos = ""
	for j in range(7):
		str_ik_pos = str_ik_pos + " J" + str(j) + ":" + str(round(jointPoses[j],3))
	
	p.removeUserDebugItem(idEEIKText)
	
	# [0.0, -0.5, 0.0] is rely to the [0.0, 0.0, 0.0] of the world
	idEEIKText = p.addUserDebugText(str_ik_pos, [-1.0, -0.5, 0.0],
				   textColor, textSize=2.0,
				   parentObjectUniqueId=kukaUid)
    
	p.stepSimulation(physicsClientId=physicsClient) # let time to update the GUI

# disconnect from the bullet environment
p.disconnect()

