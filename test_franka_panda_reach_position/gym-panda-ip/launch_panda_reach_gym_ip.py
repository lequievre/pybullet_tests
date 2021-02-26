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

def main():
    env = gym.make('panda-ip-reach-v0')
    observation = env.reset()
    
    for i_episode in range(20):
        observation = env.reset()
        for t in range(100):
            env.render()
            print(observation)
            action = env.action_space.sample()
            print("action = {0}".format(action))
            observation, reward, done, info = env.step(action)
            if done:
                print("Episode finished after {} timesteps".format(t+1))
                break
    env.close()
    

if __name__ == '__main__':
    #main()
    
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

    print("Enter to quit into the graphic window !")
    while True:
    
        # look if there is keyboard event and put it in a dictionary named keys
        keys = p.getKeyboardEvents()
    
        # 'Enter' event = 65309, if so .. break the loop
        if 65309 in keys:
            break

    p.disconnect()

