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


evaluate_policy (need to read the source code) :
==============================================

(*) example without 'return_episode_rewards' option :
  mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=100)
  
(*) example with 'return_episode_rewards' option :
 episode_rewards, episode_lengths = evaluate_policy(model, env, return_episode_rewards=True,n_eval_episodes=100)

(*) infos :
episode_rewards -> list of 'sum reward' of each episode  [sum_reward_episode1, sum_reward_episode2, ..., sum_reward_episodeN]
episode_lengths -> list of 'length/nb steps or episode time steps' of each episode [nb_steps_episode1, nb_steps_episode2, ..., nb_steps_episodeN]

mean_reward = np.mean(episode_rewards)  (mean of episode_rewards)
std_reward = np.std(episode_rewards)    (standard deviation (écart type) of episode_rewards)

( L'écart-type sert à mesurer la dispersion, ou l'étalement, d'un ensemble de valeurs autour de leur moyenne. 
Plus l'écart-type est faible, plus la population est homogène. )

(*) example of output with DDPG algo :
-> model evaluation without learning
mean_reward:-1215.51 +/- std_reward:236.55
-> model evaluation with learning
mean_reward:-605.88 +/- std_reward:100.04

(*) in case of Pendulum :
Pendulum is wrapped by the TimeLimit wrapper !
So step function returns true for done after 200 steps !
So the nb steps by episode is 200 !
So episode_lengths is a list of 200 values ! [200, 200, .... , 200]


Output of DDPG (verbos option set to 1) (need to read the source code) :
======================================================================
class BaseAlgorithm(ABC) -> stable_baselines3/common/base_class.py
^
+
+
class OffPolicyAlgorithm(BaseAlgorithm) -> stable_baselines3/common/off_policy_algorithm.py
^
+
+
class TD3(OffPolicyAlgorithm) -> stable_baselines3/td3/td3.py
^
+
+
class DDPG(TD3) -> stable_baselines3/ddpg/ddpg.py

(*) code of class BaseAlgorithm(ABC) :

def _update_learning_rate(self, optimizers: Union[List[th.optim.Optimizer], th.optim.Optimizer]) -> None:
        """
        Update the optimizers learning rate using the current learning rate schedule
        and the current progress remaining (from 1 to 0).

        :param optimizers:
            An optimizer or a list of optimizers.
        """
        # Log the current learning rate
        logger.record("train/learning_rate", self.lr_schedule(self._current_progress_remaining))

(*) Code of class OffPolicyAlgorithm(BaseAlgorithm) : 

def learn( .....):
  ...
  while self.num_timesteps < total_timesteps:
      .....
      # collect rollout
      rollout = self.collect_rollouts(
                self.env,
                n_episodes=self.n_episodes_rollout,
                n_steps=self.train_freq,
                action_noise=self.action_noise,
                callback=callback,
                learning_starts=self.learning_starts,
                replay_buffer=self.replay_buffer,
                log_interval=log_interval,
            )
       ....
       if self.num_timesteps > 0 and self.num_timesteps > self.learning_starts:
          ...
          self.train(batch_size=self.batch_size, gradient_steps=gradient_steps)


def collect_rollouts(
        self,
        env: VecEnv,
        callback: BaseCallback,
        n_episodes: int = 1,
        n_steps: int = -1,
        action_noise: Optional[ActionNoise] = None,
        learning_starts: int = 0,
        replay_buffer: Optional[ReplayBuffer] = None,
        log_interval: Optional[int] = None,
    ) -> RolloutReturn:
   # Collect experiences and store them into a 'ReplayBuffer'
   ...
   episode_rewards, total_timesteps = [], []
   total_steps, total_episodes = 0, 0
   while total_steps < n_steps or total_episodes < n_episodes:
            done = False
            episode_reward, episode_timesteps = 0.0, 0

            while not done:

                if self.use_sde and self.sde_sample_freq > 0 and total_steps % self.sde_sample_freq == 0:
                    # Sample a new noise matrix
                    self.actor.reset_noise()

                # Select action randomly or according to policy
                action, buffer_action = self._sample_action(learning_starts, action_noise)

                # Rescale and perform action
                new_obs, reward, done, infos = env.step(action)

                self.num_timesteps += 1
                episode_timesteps += 1
                total_steps += 1

                ...
                episode_reward += reward
                ...
                    self._last_original_obs, new_obs_, reward_ = self._last_obs, new_obs, reward
                ...
                replay_buffer.add(self._last_original_obs, new_obs_, buffer_action, reward_, done)
                ...
            if done:
                total_episodes += 1
                self._episode_num += 1
                episode_rewards.append(episode_reward)
                total_timesteps.append(episode_timesteps)

                if action_noise is not None:
                    action_noise.reset()

                # Log training infos
                if log_interval is not None and self._episode_num % log_interval == 0:
                    self._dump_logs()
    ...
    mean_reward = np.mean(episode_rewards) if total_episodes > 0 else 0.0
    return RolloutReturn(mean_reward, total_steps, total_episodes, continue_training)

def _dump_logs(self) -> None:
        """
        Write log.
        """
        fps = int(self.num_timesteps / (time.time() - self.start_time))
        logger.record("time/episodes", self._episode_num, exclude="tensorboard")
        if len(self.ep_info_buffer) > 0 and len(self.ep_info_buffer[0]) > 0:
            logger.record("rollout/ep_rew_mean", safe_mean([ep_info["r"] for ep_info in self.ep_info_buffer]))
            logger.record("rollout/ep_len_mean", safe_mean([ep_info["l"] for ep_info in self.ep_info_buffer]))
        logger.record("time/fps", fps)
        logger.record("time/time_elapsed", int(time.time() - self.start_time), exclude="tensorboard")
        logger.record("time/total timesteps", self.num_timesteps, exclude="tensorboard")
        if self.use_sde:
            logger.record("train/std", (self.actor.get_std()).mean().item())

        if len(self.ep_success_buffer) > 0:
            logger.record("rollout/success rate", safe_mean(self.ep_success_buffer))
        # Pass the number of timesteps for tensorboard
        logger.dump(step=self.num_timesteps)

(*) code of class TD3(OffPolicyAlgorithm) :

def train(self, gradient_steps: int, batch_size: int = 100) -> None:
  ...
  self._n_updates += gradient_steps
  logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
  logger.record("train/actor_loss", np.mean(actor_losses))
  logger.record("train/critic_loss", np.mean(critic_losses))

(*) Example (in case of pendulum, 1 episode = 200 timesteps) :
---------------------------------
| time/              |          |
|    episodes        | 50       |
|    fps             | 179      |
|    time_elapsed    | 55       |
|    total timesteps | 10000    |
| train/             |          |
|    actor_loss      | 167      |
|    critic_loss     | 111      |
|    learning_rate   | 0.001    |
|    n_updates       | 9800     |
---------------------------------

episodes -> self._episode_num  -> increment number of episodes during learn
fps -> int(self.num_timesteps / (time.time() - self.start_time))
time_elapsed -> int(time.time() - self.start_time)
total timesteps -> self.num_timesteps

actor_loss -> np.mean(actor_losses)
critic_loss -> np.mean(critic_losses)





