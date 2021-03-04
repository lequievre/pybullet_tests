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

def main():
    env = gym.make('panda-ip-reach-v0')
    observation = env.reset()
    
    for i_episode in range(20):
        observation = env.reset()
        for t in range(100):
            env.render()
            action = env.action_space.sample()
            print("action = {0}".format(action))
            observation, reward, done, info = env.step(action)
            print("observation = {0}".format(observation))
            print("reward = {0}".format(reward))
            if done:
                print("Episode finished after {} timesteps".format(t+1))
                break
    env.close()
    

if __name__ == '__main__':
    main()

