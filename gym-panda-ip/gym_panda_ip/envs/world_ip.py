"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

import os

import pybullet as p
import pybullet_data

import numpy as np
import math as m
import random


"""import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.sys.path.insert(0, currentdir)"""


class WorldIP:
    

    def __init__(self, physicsClientId=None, 
                 urdfRoot=pybullet_data.getDataPath()
                 ):
        self._physics_client_id = physicsClientId
        self._urdfRoot = urdfRoot
        
        self.table_id = None
        self.obj_id = None
        self.tray_id = None
        self.object_id = None
        
        # Add a search data path
        p.setAdditionalSearchPath(self._urdfRoot)
         
        self.reset()
    
    def reset(self):
        # Load plane
        p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), basePosition=[0,0,-0.65], physicsClientId=self._physics_client_id)
        
        # Load table, tray and object
        self.table_id = p.loadURDF(os.path.join(self._urdfRoot, "table/table.urdf"),
                              basePosition=[0.5,0,-0.65], useFixedBase=True, physicsClientId=self._physics_client_id)

        self.tray_id = p.loadURDF(os.path.join(self._urdfRoot, "tray/traybox.urdf"),basePosition=[0.65,0,0], physicsClientId=self._physics_client_id)
        
        state_object= [random.uniform(0.5,0.8),random.uniform(-0.2,0.2),0.05]
        self.object_id = p.loadURDF(os.path.join(self._urdfRoot, "random_urdfs/000/000.urdf"), basePosition=state_object, physicsClientId=self._physics_client_id)
    
    def get_state_object(self):
        return p.getBasePositionAndOrientation(self.object_id, physicsClientId=self._physics_client_id)
        
        