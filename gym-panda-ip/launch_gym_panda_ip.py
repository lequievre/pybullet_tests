"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

import gym
import gym_panda_ip
env = gym.make('panda-ip-v0')

env.reset()
for _ in range(100):
    env.render()
    obs, reward, done, info = env.step(None) # No action, just to test it like that !
    #obs, reward, done, info = env.step(
    #   env.action_space.sample()) # take a random action
env.close()

