Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr

How to test gym panda package into a virtual env :
================================================

0- Create a 'test_rl' directory at home directory :
cd ~
mkdir test_rl
cd test_rl

1- Create a virtual env named for example 've_panda' into 'test_rl' :
virtualenv ve_panda
source ve_panda/bin/activate

2- Clone the git project into 'test_rl' :
git clone https://github.com/lequievre/pybullet.git

3- Install necessary packages
cd pybullet
cd test_franka_panda_reach_position/gym-panda-ip
pip install -r requirements.txt

4- Install gym-panda-ip package on your local system :
pip install -e .

-> It will create a 'link file' named 'gym-panda-ip.egg-link' into the directory '~/test_rl/ve_panda/lib/python3.5/site-packages'

-> That file contains the directory where to find the package : '~/test_rl/pybullet/test_franka_panda/gym-panda-ip'

5- Launch pid gym test :
python launch_panda_reach_gym_ip.py

6- deactivate virtual env :
deactive



