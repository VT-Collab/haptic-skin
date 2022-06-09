import time
import numpy as np
import pickle
import argparse
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from scipy import interpolate

from utils import *

########## margins in workspace ##########
ee_home_x = 0.2
x_margin_1 = 0.5
x_margin_2 = 0.8
x_margin_3 = 1.1

Panda = TrajectoryClient()

def region(ax, start_marg, end_marg):
    ax.axvspan(start_marg, end_marg, color='#a1d99b')


X = {}
Y = {}
Z = {}
Quat = {}
n = 5

for method in ["none", "GUI", "local", "global"]:
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(12,4))
    fig.suptitle("[" + method.capitalize() + "]")

    x = {}
    y = {}
    z = {}
    quat = {}

    for user_n in range(1, n+1):
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

        x["user_" + str(user_n)] = points[0,:] - points[0,0]
        y["user_" + str(user_n)] = points[1,:]
        z["user_" + str(user_n)] = points[2,:]
        quat["user_" + str(user_n)] = quat_diff

    X[method] = x
    Y[method] = y
    Z[method] = z
    Quat[method] = quat

    ### plot user demonstrations: none ###

    for user_n in range(1, n+1):

        # height
        ax1.plot(X[method]["user_" + str(user_n)], Z[method]["user_" + str(user_n)])
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_ylim(0, 0.8)
        ax1.title.set_text('Height From Table')
        if method == "GUI":
            region(ax1, ee_home_x, x_margin_1)
        elif method == "global":
            region(ax1, ee_home_x, x_margin_1)
        elif method == "local":
            region(ax1, x_margin_1, x_margin_2)

        # orientation
        ax2.plot(X[method]["user_" + str(user_n)], Quat[method]["user_" + str(user_n)])
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Quaternion')
        ax2.set_ylim(0, 1.5)
        ax2.title.set_text('End-Effector Orientation')
        if method == "GUI":
            region(ax2, x_margin_1, x_margin_2)
        elif method == "global":
            region(ax2, x_margin_2, x_margin_3)
        elif method == "local":
            region(ax2, ee_home_x, x_margin_1)

        # distance
        ax3.plot(X[method]["user_" + str(user_n)], Y[method]["user_" + str(user_n)], label="user_" + str(user_n))
        ax3.set_xlabel('X [m]')
        ax3.set_ylabel('Y [m]')
        ax3.set_ylim(-0.5, 0.5)
        ax3.title.set_text('Distance From User')
        if method == "GUI":
            region(ax3, x_margin_2, x_margin_3)
        elif method == "global":
            region(ax3, x_margin_1, x_margin_2)
        elif method == "local":
            region(ax3, x_margin_2, x_margin_3)

    plt.tight_layout()
    plt.savefig("results_plot/" + method + ".png")



print(X.keys())

def interp(a, b):
    a = np.arange(-0.4, 0.8, 0.1)
    f = interpolate.interp1d(a, b)
    bnew = f(Xnew)
    return bnew

a = X["none"]["user_1"]

b = Y["none"]["user_1"]
c = Z["none"]["user_1"]
d = Quat["none"]["user_1"]



exit()




# handles, labels = ax3.get_legend_handles_labels()
# fig.legend(handles, labels, loc='upper right', ncol=3)


# To Does:
# plot averaged user's traj for different methods
