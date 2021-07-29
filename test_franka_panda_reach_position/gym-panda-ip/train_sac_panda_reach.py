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

from stable_baselines3 import SAC

from stable_baselines3.common.evaluation import evaluate_policy

from stable_baselines3.common.env_checker import check_env

from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv

import numpy as np

def main():

   """
   # Example with Vectorized env
   num_cpu = 4  # Number of processes to use
   my_env_kwargs={'renders': False}
   env = make_vec_env('panda-ip-reach-v0', n_envs=num_cpu, env_kwargs=my_env_kwargs)
   """

   # Example with a simple Dummy vec env
   env = gym.envs.make('panda-ip-reach-v0', renders= True)
   env = DummyVecEnv([lambda: env])


   #check_env(pandaenv)

   model = SAC(policy='MlpPolicy', env=env, verbose=1)

   """
   print("start model evaluation without learning !")
   mean_reward_before, std_reward_before = evaluate_policy(model, env, n_eval_episodes=1)
   print("end model evaluation !")
   """
   print("start model learning !")
   model.learn(total_timesteps=200000, log_interval=10)
   print("end model learning !")

   print("-> model saved !!")
   model.save("sac_panda_reach")
   """
   print("start model evaluation with learning !")
   mean_reward_after, std_reward_after = evaluate_policy(model, env, n_eval_episodes=1)
   print("end model evaluation !")
   """

   """
classstable_baselines3.ddpg.DDPG(policy, env, learning_rate=0.001, buffer_size=1000000, learning_starts=100, batch_size=100, tau=0.005, gamma=0.99, train_freq=(1, 'episode'), gradient_steps=- 1, action_noise=None, optimize_memory_usage=False, tensorboard_log=None, create_eval_env=False, policy_kwargs=None, verbose=0, seed=None, device='auto', _init_setup_model=True)
   """




if __name__ == '__main__':
    main()
