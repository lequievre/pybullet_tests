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
import numpy as np

from gym_panda_ip.envs.franka_panda_env import PandaEnv
from gym_panda_ip.envs.world_env import WorldEnv

from gym_panda_ip.envs.utils import scale_gym_data

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


        self._observation = []

        # load robot
        self._robot = PandaEnv(self._physics_client_id)

        # load world environment
        self._world = WorldEnv(self._physics_client_id,
                       obj_name=obj_name, obj_pose_rnd_std=obj_pose_rnd_std,
                       workspace_lim=self._robot.get_workspace())

        # limit robot workspace to table plane
        workspace = self._robot.get_workspace()
        workspace[2][0] = self._world.get_table_height()
        #print("table height = ", self._world.get_table_height())
        self._robot.set_workspace(workspace)

        # Define gym spaces
        self.observation_space, self.action_space = self.create_gym_spaces()

        self.seed()


    def create_gym_spaces(self):
        observation, observation_lim = self.get_extended_observation()

        observation_low = []
        observation_high = []

        for element in observation_lim:
            #print(element)
            observation_low.extend([element[0]])
            observation_high.extend([element[1]])

        # Configure the observation space
        observation_space = spaces.Box(np.array(observation_low), np.array(observation_high), dtype='float32')

        # Configure action space
        self.action_dim = self._robot.get_action_dim()
        action_bound = 1
        action_high = np.array([action_bound] * self.action_dim)
        action_space = spaces.Box(-action_high, action_high, dtype='float32')

        return observation_space, action_space


    def get_extended_observation(self):
        self._observation = []
        observation_lim = []

        # ----------------------------------- #
        # --- Robot and world observation --- #
        # ----------------------------------- #
        robot_observation, robot_observation_lim = self._robot.get_observation()
        world_observation, world_observation_lim = self._world.get_observation()

        self._observation.extend(list(robot_observation))
        self._observation.extend(list(world_observation))
        observation_lim.extend(robot_observation_lim)
        observation_lim.extend(world_observation_lim)

        return np.array(self._observation).ravel(), observation_lim


    def reset(self):
        #self.reset_simulation() TO DO

        obs, _ = self.get_extended_observation()
        scaled_obs = scale_gym_data(self.observation_space, obs)
        return scaled_obs
      

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        self._world.seed(seed)
        self._robot.seed(seed)
        return [seed]

    def step(self, action):
        # apply action on the robot
        #self.apply_action(action) TO DO

        obs, _ = self.get_extended_observation()
        scaled_obs = scale_gym_data(self.observation_space, obs)

        #done = self._termination() TO DO
        #reward = self._compute_reward() TO DO

        #return scaled_obs, np.array(reward), np.array(done), {} TO DO

    def render(self, mode="rgb_array"):
        print("PandaReachGymEnv  render !")

    def close(self):
        p.disconnect()

