"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

from gym.envs.registration import register

register(
    id='panda-ip-v0',
    entry_point='gym_panda_ip.envs:PandaIPEnv',
)
