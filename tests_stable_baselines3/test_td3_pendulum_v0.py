import gym

from stable_baselines3 import TD3

env = gym.make('Pendulum-v0')

# check env
#from stable_baselines3.common.env_checker import check_env
#check_env(env)

model = TD3.load("td3_pendulum")

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()

