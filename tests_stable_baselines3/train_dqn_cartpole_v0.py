import gym
import numpy as np

from stable_baselines3 import DQN
from stable_baselines3.dqn import MlpPolicy
from stable_baselines3.common.evaluation import evaluate_policy

env = gym.make('CartPole-v0')

model = DQN(MlpPolicy, env, verbose=1)

#model = DQN(MlpPolicy, env, seed=1423, target_update_interval =5, batch_size=16, train_freq=128, buffer_size=256, gamma=0.95, learning_rate=1e-3, verbose=1)

print("start model evaluation without learning !")
mean_reward_before, std_reward_before = evaluate_policy(model, env, n_eval_episodes=100)
print("end model evaluation !")

print("start model learning !")
model.learn(total_timesteps=10000, log_interval=4)
print("end model learning !")

print("-> model saved !!")
model.save("dqn_cartpole")

print("start model evaluation with learning !")
mean_reward_after, std_reward_after = evaluate_policy(model, env, n_eval_episodes=100)
print("end model evaluation !")

print("-> model evaluation without learning")
print(f"mean_reward:{mean_reward_before:.2f} +/- std_reward:{std_reward_before:.2f}")

print("-> model evaluation with learning")
print(f"mean_reward:{mean_reward_after:.2f} +/- std_reward:{std_reward_after:.2f}")
