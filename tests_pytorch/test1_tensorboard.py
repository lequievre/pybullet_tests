# Laurent LEQUIEVRE\n",
# Research Engineer, CNRS (France)\n",
# Institut Pascal UMR6602\n",
# laurent.lequievre@uca.fr"

# execute this python code :
# python ./test1_tensorboard.py
# After, this code create a 'runs' folder with log informations
# to see it, run this command (at the same dir of ./test1_tensorboard.py) :
# tensorboard --logdir runs --host localhost
# It's generate an URL -> http://localhost:6006/
# open it with an internet browser ans have a look to the log infos.

import math
from torch.utils.tensorboard import SummaryWriter

if __name__=="__main__":
	writer = SummaryWriter()
	funcs = {"sin": math.sin, "cos": math.cos, "tan": math.tan}
	for angle in range(-360, 360):
		angle_rad = angle * math.pi / 180
		for name, fun in funcs.items():
			val = fun(angle_rad)
			writer.add_scalar(name, val, angle)
	writer.close()
	print("fini!")
