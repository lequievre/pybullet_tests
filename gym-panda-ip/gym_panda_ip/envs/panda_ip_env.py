"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

"""import os,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print ("current_dir=" + currentdir)
os.sys.path.insert(0,currentdir)"""

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import pybullet as p
from gym_panda_ip.envs.panda_ip import PandaIP
from gym_panda_ip.envs.world_ip import WorldIP

import numpy as np
import math

MAX_EPISODE_LEN = 20*100


class PandaIPEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, pybullet_client_id):
      print("PandaIPEnv -> __init__()")
      self._timeStep = 1. / 240. # 1 / 240 seconds
      
      self.step_counter = 0
      
      self._physics_client_id = pybullet_client_id
      
      # reset the 3D OpenGL debug visualizer camera distance 
      # (between eye and camera target position), camera yaw and pitch and camera target position.
      p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2], physicsClientId=self._physics_client_id)
      
      
      self.action_space = spaces.Box(np.array([-1]*4), np.array([1]*4))
      self.observation_space = spaces.Box(np.array([-1]*5), np.array([1]*5))
        
      # disable rendering before loading objects
      p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
      
       # Load Robot Panda
      self._robot = PandaIP(self._physics_client_id, base_position=[0.0, 0.0, 0.0])
      
      # Load World (plane + table + tray + object)
      self._world = WorldIP(self._physics_client_id)
      
      # enable rendering after loading objects
      p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
      

    def step(self, action):
      print("PandaIPEnv -> step()")
      p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
      
      self._robot.apply_action(action)
      
      p.stepSimulation()
      
      state_object, _ = self._world.get_state_object()
      
      state_robot = self._robot.get_state_robot_effector()
      
      state_fingers = self._robot.get_state_fingers()
      
      if state_object[2]>0.45:
            reward = 1
            done = True
      else:
            reward = 0
            done = False

      self.step_counter += 1
    
      if self.step_counter > MAX_EPISODE_LEN:
        reward = 0
        done = True
    
      #info = {'object_position': state_object}
      info = state_object
      self.observation = state_robot + state_fingers
      return np.array(self.observation).astype(np.float32), reward, done, info
      

    def reset(self):
      print("PandaIPEnv -> reset()")
      self.step_counter = 0
      
      self.reset_simulation()
      
      self.observation = self._robot.get_observation()
      
      return self.observation
      
    def reset_simulation(self):
        print("PandaIPEnv -> reset_simulation()")
        # --- reset simulation --- #
        p.resetSimulation(physicsClientId=self._physics_client_id)
        #p.setPhysicsEngineParameter(numSolverIterations=150, physicsClientId=self._physics_client_id)
        #p.setTimeStep(self._timeStep, physicsClientId=self._physics_client_id)
        
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
        
        p.setGravity(0, 0, -9.8, physicsClientId=self._physics_client_id)

        # --- reset robot --- #
        self._robot.reset()

        # --- reset world --- #
        self._world.reset()
            
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

    def render(self, mode='human'):
        print("PandaIPEnv -> render()")
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
                                                         distance=.7,
                                                         yaw=90,
                                                         pitch=-70,
                                                         roll=0,
                                                         upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                  aspect=float(960) /720,
                                                  nearVal=0.1,
                                                  farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960,
                                           height=720,
                                           viewMatrix=view_matrix,
                                           projectionMatrix=proj_matrix,
                                           renderer=p.ER_BULLET_HARDWARE_OPENGL)
        
        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720,960, 4))
        
        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):
        print("PandaIPEnv -> close()")
    
    def _get_state(self):
        return self.observation
      
    
        
    