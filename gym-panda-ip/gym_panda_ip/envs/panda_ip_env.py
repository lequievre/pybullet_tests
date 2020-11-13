"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

import gym
from gym import error, spaces, utils
from gym.utils import seeding

class PandaIPEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
      print("PandaIPEnv -> __init__()")

    def step(self, action):
      print("PandaIPEnv -> step()")
      return None, None, None, None

    def reset(self):
      print("PandaIPEnv -> reset()")

    def render(self, mode='human'):
      print("PandaIPEnv -> render()")

    def close(self):
      print("PandaIPEnv -> close()")
