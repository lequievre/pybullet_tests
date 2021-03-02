#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

import gym
import gym_panda_ip
import pybullet as p

import time

if __name__ == '__main__':
    
    
    env = gym.make('panda-ip-reach-v0')
    print('action space -> ')

    a = env.action_space
    print('action = ', a)                    
    print('shape = ', a.shape)
    print('low = ', a.low)            
    print('high = ', a.high)

    """
    action space -> 
	action =  Box(7,)
	shape =  (7,)
	low =  [-1. -1. -1. -1. -1. -1. -1.]
	high =  [1. 1. 1. 1. 1. 1. 1.]

    Actions are normalized between -1 to 1 (7 joint values)
    This fixed range of [-1,1] helps in stabilising the training in the neural networks
    """
  
    print('observation space -> ')
    o = env.observation_space
    print('observation = ', o)                    
    print('shape = ', o.shape)
    print('low = ', o.low)
    print('high = ', o.high)               
    
    """
   	observation space -> 
	observation =  Box(21,)
	shape =  (21,)
	low =  [ 0.3       -0.3        0.625     -3.1415927 -3.1415927 -3.1415927
	 -2.9671    -1.8326    -2.9671    -3.1416    -2.9671    -0.0873
	 -2.9671     0.         0.         0.3       -0.3        0.625
	 -3.1415927 -3.1415927 -3.1415927]
	high =  [0.65      0.3       1.5       3.1415927 3.1415927 3.1415927 2.9671
	 1.8326    2.9671    0.        2.9671    3.8223    2.9671    0.04
	 0.04      0.65      0.3       0.925     3.1415927 3.1415927 3.1415927]

    !! table height =  0.625 -> change workspace limit z, so end eff limit z !!
    !! [0.65, 1.5] become [0.625, 0.625 + 0.3 = 0.925]

    Observation = 
    end_eff_pos_x, end_eff_pos_y, end_eff_pos_z, end_eff_euler_x, end_eff_euler_y, end_eff_euler_z,
    panda_joint1_value, panda_joint2_value, panda_joint3_value, panda_joint3_value, panda_joint4_value, 
    panda_joint5_value, panda_joint6_value, panda_joint7_value,
    obj_pos_x, obj_pos_y, obj_pos_z, obj_euler_x, obj_euler_y, obj_euler_z

    Observation limit =
    [0.3, 0.65], [-0.3, 0.3], [0.65, 1.5], [-m.pi, m.pi], [-m.pi, m.pi], [-m.pi, m.pi],

    [-2.9671, 2.9671], [-1.8326, 1.8326], [-2.9671, 2.9671], [-3.1416, 0.0], [-2.9671, 2.9671],
    [-0.0873, 3.8223], [-2.9671, 2.9671], [0.0, 0.04], [0.0, 0.04],

    [0.3, 0.65], [-0.3, 0.3], [0.65, 1.5], [-m.pi, m.pi], [-m.pi, m.pi], [-m.pi, m.pi]
    
    """

    
    observation_scaled = env.reset()
    print('observation scaled after reset = ', observation_scaled)

    # print robot observation
    observation, _ = env._robot.get_observation()
    list_items = list(env._robot._joint_name_to_index.items())
    for i in range(len(list_items)):
       print(list_items[i],"->",observation[6+i])
    

    """
    observation scaled after reset =  [-6.07307340e-01 -5.97678789e-02 -1.91250055e-02  9.33127154e-01
	 -5.70407134e-02 -2.69678429e-01  0.00000000e+00 -2.94663320e-01
	  0.00000000e+00 -6.55207594e-01 -1.01108830e-01  6.77818663e-02
	  3.37029433e-01  2.23517422e-08  2.23517422e-08 -2.23669335e-01
	  4.71404502e-02 -5.33333352e-01  0.00000000e+00  0.00000000e+00
	  2.49999993e-01]
    """

    timestep = 1./240.
    physics_client_id = env._physics_client_id

    print("Enter to quit into the graphic window !")
    while True:
    
        # look if there is keyboard event and put it in a dictionary named keys
        keys = p.getKeyboardEvents()
    
        # 'Enter' event = 65309, if so .. break the loop
        if 65309 in keys:
            break

        if 97 in keys: # (with letter 'a' = 97)
            env.apply_action([0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        elif 98 in keys: # (with letter 'b' = 98)
            env.apply_action([0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
        elif 99 in keys: # (with letter 'c' = 99)
            env.apply_action([0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0])
        elif 100 in keys: # (with letter 'd' = 100)
            env.apply_action([0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0])
        elif 101 in keys: # (with letter 'e' = 101)
            env.apply_action([0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        elif 102 in keys: # (with letter 'f' = 102)
            env.apply_action([0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0])
        elif 103 in keys: # (with letter 'g' = 103)
            env.apply_action([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2])
        elif keys.get(105): # (with letter 'i' = 105)
            # print robot observation
            observation, _ = env.get_extended_observation()
            list_items = list(env._robot._joint_name_to_index.items())
            for i in range(len(list_items)):
               print(list_items[i],"->",observation[6+i])

            # Let the world run for a bit
            for _ in range(100):
               p.stepSimulation(physicsClientId=physics_client_id )
            
        # let the simulation to redraw	   
        p.stepSimulation(physicsClientId=physics_client_id )
        #time.sleep(timestep)
            

    p.disconnect()

