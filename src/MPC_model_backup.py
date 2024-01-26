#!/usr/bin/env python
import rospy
import numpy as np
import scipy.optimize as opt
import numpy.matlib
import math
import random
from std_msgs.msg import Int32MultiArray, Int16





nrows = 4
ncols = 4
meter_per_sector_length = 1

T = 30
Tp = 5

ns = nrows * ncols

#no of defenders
nf = 3

#no of attackers 
ne = 1

#cost function parameter
alphaf = 0.99; betaf = 1-alphaf
alphae = 0.001; betae = 1-alphae

#neighbourhood matrices
Bin, Bout, N, Neigh = Bmatrix(nrows, ncols)
B = Bin - Bout

Aeq = DynamicConstraints(Bin, Bout, Tp, ns)

A = FlowConstraints(Bin, Bout, Tp, ns)

num_games = 50;num_lost = 0
num_iterations = np.zeros(1, num_games)
