Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr

Stable Baselines3 (SB3) is a set of reliable implementations of reinforcement learning algorithms in PyTorch. 
It is the next major version of Stable Baselines.

Github repository: https://github.com/DLR-RM/stable-baselines3

Docs : https://stable-baselines3.readthedocs.io/en/master/index.html


How to install Stable Baselines3 (SB3)
======================================

Stable-Baselines3 requires python 3.6+.

The default python versions are 2.7 and 3.5 on Ubuntu 16.04.

1 -> On Ubuntu 16.04 (and 18.04), install python 3.9 :

sudo add-apt-repository ppa:deadsnakes/ppa

sudo apt update

sudo apt install python3.9
sudo apt install python3.9-venv


2 -> Create a virtual environment :

=> creation of a directory 'my_envs' that will contain the different virtual environments :

cd ~
mkdir my_envs
cd my_envs

=> create a virtual env named 've_stable_baselines3' with the default python version '/usr/bin/python3.9',
=> this command create a directory named 've_stable_baselines3' into the current directory :

virtualenv -p /usr/bin/python3.9 ve_stable_baselines3

=> activate the virtual env :

source ve_stable_baselines3/bin/activate

=> install the packages 'stable_baselines3' and 'tensorboard' (for logging) :

pip3.9 install stable-baselines3
pip3.9 install tensorboard

3 -> launch a training and test of ddpg (for example) :

DDPG :

python train_ddpg_pendulum_v0.py
(model is saved in a zip file named 'ddpg_pendulum.zip')

python python test_ddpg_pendulum_v0.py
(load model 'ddpg_pendulum.zip')

TD3 :
python train_td3_pendulum_v0.py

python test_td3_pendulum_v0.py

DQN :
python train_dqn_cartpole_v0.py

python test_dqn_cartpole_v0.py


How to use "tensorboard" :
=========================

-> Add the option 'tensorboard_logs' in the constructor :
model = DDPG('MlpPolicy', env, action_noise=action_noise, verbose=1, tensorboard_log="./ddpg_pendulum_tensorboard/")

-> create a directory named 'ddpg_pendulum_tensorboard' (for this example).

-> launch this command into another terminal window (don't forget to activate the virtual env before !)
tensorboard --logdir ./ddpg_pendulum_tensorboard --host localhost

that command return an url : http://localhost:6006/
(copy and paste this url in an internet browser)


evaluate_policy :
================

(*) example without 'return_episode_rewards' option :
  mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=100)
  
(*) example with 'return_episode_rewards' option :
 episode_rewards, episode_lengths = evaluate_policy(model, env, return_episode_rewards=True,n_eval_episodes=100)

(*) infos :
episode_rewards -> list of 'sum reward' of each episode  [sum_reward_episode1, sum_reward_episode2, ..., sum_reward_episodeN]
episode_lengths -> list of 'length/nb steps' of each episode [nb_steps_episode1, nb_steps_episode2, ..., nb_steps_episodeN]

mean_reward = np.mean(episode_rewards)  (mean of episode_rewards)
std_reward = np.std(episode_rewards)    (standard deviation (écart type) of episode_rewards)

( L'écart-type sert à mesurer la dispersion, ou l'étalement, d'un ensemble de valeurs autour de leur moyenne. 
Plus l'écart-type est faible, plus la population est homogène. )


(*) example of output with DDPG algo :

-> model evaluation without learning
mean_reward:-1215.51 +/- std_reward:236.55
-> model evaluation with learning
mean_reward:-605.88 +/- std_reward:100.04






