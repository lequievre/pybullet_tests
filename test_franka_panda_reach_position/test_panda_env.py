from envs.franka_panda_env import pandaEnv
import pybullet as p

#physics_client_id = p.connect(p.DIRECT)
physics_client_id = p.connect(p.GUI)
p.resetSimulation()

# Set Gravity to the environment
p.setGravity(0, 0, -9.81, physics_client_id)

# Enable keyboard events
p.configureDebugVisualizer( p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1 )

# Set num of joints
numControlledJoints = 7
robot = pandaEnv(physics_client_id, joint_action_space=numControlledJoints)
print("lower limits =", robot.list_lower_limits)
print("upper limits =", robot.list_upper_limits)
print("ranges =", robot.list_ranges)
print("rest pos =", robot.list_rest_pos)
 
robot.debug_gui()

while True:
    
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, physicsClientId=physics_client_id)
    
    # look if there is keyboard event and put it in a dictionary named keys
    keys = p.getKeyboardEvents()
    
    # 'Enter' event = 65309, if so .. break the loop
    if 65309 in keys:
      break

    # let the simulation to redraw	   
    p.stepSimulation(physics_client_id)

p.disconnect()

