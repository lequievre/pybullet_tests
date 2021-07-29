"""
Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr
"""

from gym.envs.registration import register

register(
    id='panda-ip-reach-v0',
    entry_point='gym_panda_ip.envs:PandaReachGymEnv', kwargs={'action_repeat' : 1,
                 'obj_name' : 'cube_small',
                 'renders' : True,
                 'max_steps' : 1000,
                 'obj_pose_rnd_std' : 0}
)

