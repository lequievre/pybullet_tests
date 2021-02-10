import gym

from stable_baselines3 import DDPG

env = gym.make('Pendulum-v0')

model = DDPG.load("ddpg_pendulum")

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()

