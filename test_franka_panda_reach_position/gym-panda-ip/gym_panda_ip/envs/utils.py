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

import numpy as np

def scale_gym_data(data_space, data):
    """
    Rescale the gym data from [low, high] to [-1, 1]
    (no need for symmetric data space)

    :param data_space: (gym.spaces.box.Box)
    :param data: (np.ndarray)
    :return: (np.ndarray)

    x = [ (x - min(x)) / (max(x) - min(x)) - 0.5] * 2

    x = 2 * [ (x - min(x)) / (max(x) - min(x))] - 1

    """

    assert data.shape == data_space.shape

    low, high = data_space.low, data_space.high
    return 2.0 * ((data - low) / (high - low)) - 1.0

def goal_distance(a: np.ndarray, b: np.ndarray):
    if not a.shape == b.shape:
        raise AssertionError("goal_distance(): shape of points mismatch")
    return np.linalg.norm(a - b, axis=-1)
