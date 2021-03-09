import gym

from stable_baselines3 import SAC

env = gym.make('Pendulum-v0')

# check env
#from stable_baselines3.common.env_checker import check_env
#check_env(env)

model = SAC.load("sac_pendulum")

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()

