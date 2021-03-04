#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

#import gym
#import gym_panda_ip

from  gym_panda_ip.envs.franka_panda_reach_gym_env import PandaReachGymEnv

from stable_baselines3 import DDPG
from stable_baselines3.common.env_util import make_vec_env

from stable_baselines3.common.vec_env import DummyVecEnv

def main():
    """
    # Example with a simple Dummy vec env
    env = gym.envs.make('panda-ip-reach-v0', renders= True)
    env = DummyVecEnv([lambda: env])
    """
    print("Env created !")

    env = PandaReachGymEnv(renders=True)

    env.render(mode='rgb_array')

    model = DDPG.load("ddpg_panda_reach")
    print("model loaded !")

    while True:
       obs, done = env.reset(), False
       print("===================================")
       print("obs")
       print(obs)
       episode_rew = 0
       while not done:
          env.render(mode='rgb_array')
          action, _states = model.predict(obs)
          obs, rew, done, info = env.step(action)
          episode_rew += rew
       print("Episode reward", episode_rew)

if __name__ == '__main__':
    main()
