import gym
import numpy as np

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv

from stable_baselines3.common.evaluation import evaluate_policy

env = gym.make('Pendulum-v0')
env = DummyVecEnv([lambda: env])

model = SAC('MlpPolicy', env, verbose=1)

print("start model evaluation without learning !")
mean_reward_before, std_reward_before = evaluate_policy(model, env, n_eval_episodes=100)
print("end model evaluation !")

print("start model learning !")
model.learn(total_timesteps=10000, log_interval=10)
print("end model learning !")

print("-> model saved !!")
model.save("sac_pendulum")

print("start model evaluation with learning !")
mean_reward_after, std_reward_after = evaluate_policy(model, env, n_eval_episodes=100)
print("end model evaluation !")


print("-> model evaluation without learning")
print(f"mean_reward:{mean_reward_before:.2f} +/- std_reward:{std_reward_before:.2f}")

print("-> model evaluation with learning")
print(f"mean_reward:{mean_reward_after:.2f} +/- std_reward:{std_reward_after:.2f}")


"""list_per_episode_rewards, per_episode_lengths = evaluate_policy(model, env, return_episode_rewards=True,n_eval_episodes=100)
print(list_per_episode_rewards)
print(per_episode_lengths)"""

