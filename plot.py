import time
import numpy as np
import pickle
import argparse
import matplotlib.pyplot as plt
from pyquaternion import Quaternion

from utils import *


Panda = TrajectoryClient()

X = {}
Y = {}
Z = {}
Quat = {}

for method in ["none", "GUI", "local", "global"]:

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(12,4))
    fig.suptitle("[" + method.capitalize() + "]")

    x = {}
    y = {}
    z = {}
    quat = {}

    for user_n in range(1, 4):

        filename = "data/demos/user_" + str(user_n) + "_" + method +  ".pkl"
        file = open(filename, "rb")
        data = pickle.load(file)

        ee = data["ee positions"]
        points = np.array(ee).T

        R = data["rotation matrix"]
        quat_diff = []
        for r in R:
            quat_diff.append(Quaternion.absolute_distance(Panda.rot2quat(R_desire),
                                                          Panda.rot2quat(r)))

        x["user_" + str(user_n)] = points[0,:]
        y["user_" + str(user_n)] = points[1,:]
        z["user_" + str(user_n)] = points[2,:]
        quat["user_" + str(user_n)] = quat_diff

    X[method] = x
    Y[method] = y
    Z[method] = z
    Quat[method] = quat


    ### plot user demonstrations: none ###
    ee_home_x = X[method]["user_" + str(user_n)][0]

    for user_n in range(1, 4):

        # segment 1
        ax1.plot(X[method]["user_" + str(user_n)], Z[method]["user_" + str(user_n)])
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_ylim(0, 0.8)
        ax1.axvspan(ee_home_x, x_margin_1, color='#a1d99b')
        ax1.title.set_text('Segment 1')

        # segment 2
        ax2.plot(X[method]["user_" + str(user_n)], Quat[method]["user_" + str(user_n)])
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Quaternion')
        ax2.set_ylim(0, 1.5)
        ax2.axvspan(x_margin_1, x_margin_2, color='#a1d99b')
        ax2.title.set_text('Segment 2')

        # segment 3
        ax3.plot(X[method]["user_" + str(user_n)], Y[method]["user_" + str(user_n)], label="user_" + str(user_n))
        ax3.set_xlabel('X [m]')
        ax3.set_ylabel('Y [m]')
        ax3.set_ylim(-0.5, 0.8)
        ax3.axvspan(x_margin_2, x_margin_3, color='#a1d99b')
        ax3.title.set_text('Segment 3')



    handles, labels = ax3.get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', )

    plt.tight_layout()
    plt.savefig("results_plot/" + method + ".png")

# plt.show()


# To Does:
# plot averaged user's traj for different methods
# legends
