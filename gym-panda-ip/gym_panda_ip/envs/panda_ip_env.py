"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

import os,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print ("current_dir=" + currentdir)
os.sys.path.insert(0,currentdir)

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import pybullet as p
from gym_panda_ip.envs.panda_ip import pandaIP


class PandaIPEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, pybullet_client_id):
      print("PandaIPEnv -> __init__()")
      self._timeStep = 1. / 240. # 1 / 240 seconds
      
      self._physics_client_id = pybullet_client_id
      

    def step(self, action):
      print("PandaIPEnv -> step()")
      return None, None, None, None

    def reset(self):
      print("PandaIPEnv -> reset()")
      p.resetDebugVisualizerCamera(2.5, 90, -60, [0.52, -0.2, -0.33], physicsClientId=self._physics_client_id)
      
      p.resetSimulation()
      
      # Load robot
      self._robot = pandaIP(self._physics_client_id)

    def render(self, mode='human'):
      print("PandaIPEnv -> render()")

    def close(self):
      print("PandaIPEnv -> close()")
      
