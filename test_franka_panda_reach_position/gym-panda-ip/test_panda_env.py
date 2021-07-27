from gym_panda_ip.envs.franka_panda_env import PandaEnv
from gym_panda_ip.envs.world_env import WorldEnv

import pybullet as p

#physics_client_id = p.connect(p.DIRECT)
physics_client_id = p.connect(p.GUI)
p.resetSimulation()

p.resetDebugVisualizerCamera(2.5, 90, -60, [0.52, -0.2, -0.33], physicsClientId=physics_client_id)

# Set Gravity to the environment
p.setGravity(0, 0, -9.81, physics_client_id)

# Enable keyboard events
p.configureDebugVisualizer( p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1 )

robot = PandaEnv(physics_client_id)
print("lower limits =", robot.list_lower_limits)
print("upper limits =", robot.list_upper_limits)
print("ranges =", robot.list_ranges)
print("rest pos =", robot.list_rest_pos)


world = WorldEnv(physics_client_id)
print("world workspace limit = ", world.get_workspace())

workspace_robot = robot.get_workspace()
print("robot workspace limit (before table) = ", robot.get_workspace())

print("table height = ", world.get_table_height())

workspace_robot[2][0] = world.get_table_height()
robot.set_workspace(workspace_robot)

print("robot workspace limit (after table) = ", robot.get_workspace())

robot.debug_gui()
#world.debug_gui()

robot_observation, robot_observation_lim = robot.get_observation()
print("robot observation = ", robot_observation)
print("robot observation lim = ", robot_observation_lim)

world_observation, world_observation_lim = world.get_observation()
print("world observation = ", world_observation)
print("world observation lim = ", world_observation_lim)


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

