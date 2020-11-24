Laurent LEQUIEVRE
Research Engineer, CNRS (France)
Institut Pascal UMR6602
laurent.lequievre@uca.fr

* gym-panda-ip package

forked from : https://github.com/mahyaret/gym-panda


How to test gym panda package :

# Create a virtual env
virtualenv ve_panda
source ve_panda/bin/activate

# clone the git project
git clone https://github.com/lequievre/pybullet.git

# Install necessary packages
cd gym-panda-ip
pip install -r requirements.txt

# Install gym-panda-ip package on your local system
pip install -e .

# launch pid gym test
python launch_pid_gym_panda_ip.py
 





