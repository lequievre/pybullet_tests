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

from gym_panda_ip.envs.utils import scale_gym_data, goal_distance

import time

class PandaReachGymEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'],
    'video.frames_per_second': 50 }

    def __init__(self,
                 action_repeat=1,
                 obj_name='duck_vhacd',
                 renders=False,
                 max_steps=1000,
                 obj_pose_rnd_std=0):

        print("Start PandaReachGymEnv  init !")
        self._renders = renders

        if self._renders:
            #self._physics_client_id = p.connect(p.SHARED_MEMORY)
            #if self._physics_client_id < 0:
            #    self._physics_client_id = p.connect(p.GUI)
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

        self.terminated = 0

        self._target_dist_min = 0.03
        self._target_threshold = 0.001

        self._max_steps = max_steps

        self._env_step_counter = 0

        self._action_repeat = 1

        self._timeStep = 1. / 240.

        self.seed()

        print("End PandaReachGymEnv  init !")


    def create_gym_spaces(self):
        observation, observation_lim = self.get_extended_observation()

        observation_low = []
        observation_high = []

        for element in observation_lim:
            #print(element)
            observation_low.extend([element[0]])
            observation_high.extend([element[1]])

        #print('create_gym_spaces -> ', len(observation_low), len(observation_high))
        # Configure the observation space
        observation_space = spaces.Box(np.array(observation_low), np.array(observation_high),dtype=np.float32)

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
        self.reset_simulation()

        obs, _ = self.get_extended_observation()
        #scaled_obs = scale_gym_data(self.observation_space, obs)
        #print('reset ->', scaled_obs)
        #print(self.observation_space.shape)
        #return scaled_obs
        return obs

    def reset_simulation(self):
        self.terminated = 0
        self._env_step_counter = 0

        # --- reset simulation --- #
        p.resetSimulation(physicsClientId=self._physics_client_id)
        p.setPhysicsEngineParameter(numSolverIterations=150, physicsClientId=self._physics_client_id)
        p.setTimeStep(self._timeStep, physicsClientId=self._physics_client_id)
        
        p.setGravity(0, 0, -9.8, physicsClientId=self._physics_client_id)

        # --- reset robot --- #
        self._robot.reset()

        # Let the world run for a bit
        for _ in range(100):
            p.stepSimulation(physicsClientId=self._physics_client_id)

        # --- reset world --- #
        self._world.reset()

        # Let the world run for a bit
        for _ in range(100):
            p.stepSimulation(physicsClientId=self._physics_client_id)

        # --- draw some reference frames in the simulation for debugging --- #
        self._robot.debug_gui()
        self._world.debug_gui()
        p.stepSimulation(physicsClientId=self._physics_client_id)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        self._world.seed(seed)
        self._robot.seed(seed)
        return [seed]

    def step(self, action):
        # apply action on the robot
        self.apply_action(action)

        observation, _ = self.get_extended_observation()
        #scaled_obs = scale_gym_data(self.observation_space, observation)

        done = self._termination()
        reward = self._compute_reward()

        return observation, reward, done, {}

    def render(self, mode="rgb_array"):
        if mode != "rgb_array":
            return np.array([])

        base_pos, _ = p.getBasePositionAndOrientation(self._robot.robot_id,
                                                            physicsClientId=self._physics_client_id)

        cam_dist = 1.3
        cam_yaw = 180
        cam_pitch = -40
        RENDER_HEIGHT = 720
        RENDER_WIDTH = 960

        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=base_pos,
                                                                distance=cam_dist,
                                                                yaw=cam_yaw,
                                                                pitch=cam_pitch,
                                                                roll=0,
                                                                upAxisIndex=2,
                                                                physicsClientId=self._physics_client_id)

        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
                                                         nearVal=0.1, farVal=100.0,
                                                         physicsClientId=self._physics_client_id)

        (_, _, px, _, _) = p.getCameraImage(width=RENDER_WIDTH, height=RENDER_HEIGHT,
                                                  viewMatrix=view_matrix,
                                                  projectionMatrix=proj_matrix,
                                                  renderer=p.ER_BULLET_HARDWARE_OPENGL,
                                                  physicsClientId=self._physics_client_id)
        # renderer=self._p.ER_TINY_RENDERER)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (RENDER_HEIGHT, RENDER_WIDTH, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):
        p.disconnect()

    
    def _termination(self):
        robot_observation, _ = self._robot.get_observation()
        world_observation, _ = self._world.get_observation()
        d = goal_distance(np.array(robot_observation[:3]), np.array(world_observation[:3]))

        if d <= self._target_dist_min:
            self.terminated = 1
            print('------------->>> success!')
            print('final reward')
            print(self._compute_reward())

            return np.float32(1.0)

        if self.terminated or self._env_step_counter > self._max_steps:
            return True

        return False

    def _compute_reward(self):
        robot_observation, _ = self._robot.get_observation()
        world_observation, _ = self._world.get_observation()
        
        #self._target_dist_min = 0.03
        #self._target_threshold = 0.001

        # _e => effector
        x_e = robot_observation[0]
        y_e = robot_observation[1]
        z_e = robot_observation[2]
        
        # _o => object
        x_o = world_observation[0]
        y_o = world_observation[1]
        z_o = world_observation[2]
      
        d = goal_distance(np.array(robot_observation[:3]), np.array(world_observation[:3]))

        reward = 0

        if ((abs(x_e-x_o) <= self._target_threshold) 
        and (abs(y_e-y_o) <= self._target_threshold) 
        and (abs(z_e-z_o-self._target_dist_min) <= self._target_threshold)):
            reward = 1
        elif ((z_o + self._target_dist_min - self._target_threshold) > z_e):
            reward = -10
        else:
            reward = -d

        return reward


    def apply_action(self, action):
        # process action and send it to the robot

        action = scale_gym_data(self.action_space, np.array(action))

        
        robot_observation, _ = self._robot.get_observation()
        action *= 0.05

        nb_total_joints = len(self._robot._joint_name_to_index.items())  # arm  + fingers
        nb_joints_to_control = self._robot.get_action_dim()  # only arm

        # -1 is the index of the last element of an observation
        new_action = np.add(robot_observation[-nb_total_joints: -(nb_total_joints - nb_joints_to_control)], action)

        # -------------------------- #
        # --- send pose to robot --- #
        # -------------------------- #
        self._robot.apply_action(new_action)
        p.stepSimulation(physicsClientId=self._physics_client_id)
        time.sleep(self._timeStep)

        self._env_step_counter += 1

