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
    main()

