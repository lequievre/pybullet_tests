#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""
from  gym_panda_ip.envs.franka_panda_reach_gym_env import PandaReachGymEnv

from stable_baselines3 import DDPG
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

from stable_baselines3.common.env_checker import check_env



def main():

   pandaenv = PandaReachGymEnv(renders=False)

   check_env(pandaenv)


if __name__ == '__main__':
    main()
