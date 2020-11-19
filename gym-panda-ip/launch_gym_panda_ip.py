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
    
    for i_episode in range(20):
        observation = env.reset()
        for t in range(100):
            env.render()
            print(observation)
            action = env.action_space.sample()
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





