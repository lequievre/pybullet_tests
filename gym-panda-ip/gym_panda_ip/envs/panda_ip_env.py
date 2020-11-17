"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import pybullet as p
from gym_panda_ip.envs.panda_ip import pandaIP


class PandaIPEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
      print("PandaIPEnv -> __init__()")
      self._timeStep = 1. / 240.
      
      print("PandaIPEnv -> before connect()")
      
      
      self._physics_client_id = p.connect(p.SHARED_MEMORY)
      if self._physics_client_id < 0:
          self._physics_client_id = p.connect(p.GUI)
          
      p.resetDebugVisualizerCamera(2.5, 90, -60, [0.52, -0.2, -0.33], physicsClientId=self._physics_client_id)
      
      #self._physics_client_id = p.connect(p.GUI)
      #p.resetSimulation()
      
      print("PandaIPEnv -> after connect()")
      
      # Load robot
      self._robot = pandaIP(self._physics_client_id)


    def step(self, action):
      print("PandaIPEnv -> step()")
      return None, None, None, None

    def reset(self):
      print("PandaIPEnv -> reset()")
      p.resetSimulation()

    def render(self, mode='human'):
      print("PandaIPEnv -> render()")

    def close(self):
      print("PandaIPEnv -> close()")
