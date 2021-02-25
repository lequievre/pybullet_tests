#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr

forked from :

Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
This software may be modified and distributed under the terms of the
LGPL-2.1+ license.

"""

import gym
from gym import spaces
from gym.utils import seeding

import pybullet as p

from gym_panda_ip.envs.franka_panda_env import PandaEnv
from gym_panda_ip.envs.world_env import WorldEnv

class PandaReachGymEnv(gym.Env):

    def __init__(self,
                 action_repeat=1,
                 obj_name='duck_vhacd',
                 renders=False,
                 max_steps=1000,
                 obj_pose_rnd_std=0):

        print("PandaReachGymEnv  init !")
        self._renders = renders

        if self._renders:
            """self._physics_client_id = p.connect(p.SHARED_MEMORY)
            if self._physics_client_id < 0:
                self._physics_client_id = p.connect(p.GUI)"""
            self._physics_client_id = p.connect(p.GUI)
            p.resetDebugVisualizerCamera(2.5, 90, -60, [0.52, -0.2, -0.33], physicsClientId=self._physics_client_id)
        else:
            self._physics_client_id = p.connect(p.DIRECT)

        # load robot
        self._robot = PandaEnv(self._physics_client_id)

        # load world environment
        self._world = WorldEnv(self._physics_client_id,
                       obj_name=obj_name, obj_pose_rnd_std=obj_pose_rnd_std,
                       workspace_lim=self._robot.get_workspace())

        # limit robot workspace to table plane
        workspace = self._robot.get_workspace()
        workspace[2][0] = self._world.get_table_height()
        self._robot.set_workspace(workspace)

        self.seed()


    def reset(self):
        print("PandaReachGymEnv  reset !")
      

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        self._world.seed(seed)
        self._robot.seed(seed)
        return [seed]

    def step(self, action):
        print("PandaReachGymEnv  step !")

    def render(self, mode="rgb_array"):
        print("PandaReachGymEnv  render !")

    def close(self):
        p.disconnect()

