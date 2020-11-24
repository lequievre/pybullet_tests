Laurent LEQUIEVRE<br/>
Research Engineer, CNRS (France)<br/>
Institut Pascal UMR6602<br/>
laurent.lequievre@uca.fr<br/>

* RL on pybullet environment

-> see gym-panda-ip directory

* Simple grasp test using pybullet with a robot franka emika panda :

python hello_world_franka.py

Grasp an object in a trailbox (robot and trailbox are on a table)
keyboard keys used:
    - 'i' to set the panda to an initial position
    - 'p' to set the panda to a pre grasp position
    - 'd' to descend vertically the panda
    - 'h' to grasp the object (sorry but the letter 'g' is a shortcut in pybullet for removing the ground)
    - 'r' to remove the object from the trailbox
    - 'Enter' to leave the pybullet environment
