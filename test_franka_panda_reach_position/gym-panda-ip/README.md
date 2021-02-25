Laurent LEQUIEVRE<br/>
Research Engineer, CNRS (France)<br/>
Institut Pascal UMR6602<br/>
laurent.lequievre@uca.fr<br/>

* How to test gym panda package into a virtual env

0- Create a 'test_rl' directory at home directory :<br/>
cd ~<br/>
mkdir test_rl<br/>
cd test_rl<br/>

1- Create a virtual env named for example 've_panda' into 'test_rl' :<br/>
virtualenv ve_panda<br/>
source ve_panda/bin/activate<br/>

2- Clone the git project into 'test_rl' :<br/>
git clone https://github.com/lequievre/pybullet.git<br/>

3- Install necessary packages<br/>
cd pybullet<br/>
cd test_franka_panda_reach_position/gym-panda-ip<br/>
pip install -r requirements.txt<br/>

4- Install gym-panda-ip package on your local system :<br/>
pip install -e .<br/>

-> It will create a 'link file' named 'gym-panda-ip.egg-link' into the directory '~/test_rl/ve_panda/lib/python3.5/site-packages'<br/>

-> That file contains the directory where to find the package : '~/test_rl/pybullet/test_franka_panda/gym-panda-ip'<br/>

5- Launch pid gym test :<br/>
python launch_panda_reach_gym_ip.py<br/>

6- deactivate virtual env :<br/>
deactive<br/>



