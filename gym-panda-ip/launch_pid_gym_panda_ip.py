"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

import pybullet as p


client_id = p.connect(p.GUI)    

from gym_panda_ip.envs.panda_ip_env import PandaIPEnv


def main():

    env = PandaIPEnv(client_id)
    
    done = False
    error = 0.01
    fingers = 1
    info = [0.7, 0, 0.1]
    cc = info[0]
    
    k_p = 0.5
    k_d = 0.1
    dt = 1./240. # the default timestep in pybullet is 240 Hz  
    t = 0
    
    for i_episode in range(20):
        observation = env.reset()
        fingers = 1
        for t in range(100):
            print("t = {0}, observatio = {1}, info = {2}".format(t, observation, info))
            env.render()
            dx = info[0]-observation[0]
            dy = info[1]-observation[1]
            target_z = info[2] 
            if abs(dx) < error and abs(dy) < error and abs(dz) < error:
                fingers = 0
            if (observation[3]+observation[4])<error+0.02 and fingers==0:
                target_z = 0.5
            dz = target_z-observation[2]
            pd_x = k_p*dx + k_d*dx/dt
            pd_y = k_p*dy + k_d*dy/dt
            pd_z = k_p*dz + k_d*dz/dt
            action = [pd_x,pd_y,pd_z,fingers]
            observation, reward, done, info = env.step(action)
            if done:
                print("Episode finished after {} timesteps".format(t+1))
                break
    env.close()
    
    
if __name__ == '__main__':
    main()
    
    print("Enter to quit !")
    while True:
    
        # look if there is keyboard event and put it in a dictionary named keys
        keys = p.getKeyboardEvents()
    
        # 'Enter' event = 65309, if so .. break the loop
        if 65309 in keys:
            break

    p.disconnect()   
    
    
    
    
    
    





