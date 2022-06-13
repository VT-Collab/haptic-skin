import time
import numpy as np
import pickle
import argparse
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from scipy import interpolate
from cycler import cycler
from utils import *


Panda = TrajectoryClient()

def region(ax, start_marg, end_marg):
    ax.axvspan(start_marg, end_marg, color='#a1d99b')

X = {}
Y = {}
Z = {}
Quat = {}
n = 4
colors = np.range(0, 1, 0.1)
print(colors)
exit()

for method in ["none", "GUI", "local", "global"]:
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(12,4))
    fig.suptitle(method.upper())

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

    ### plot user demonstrations ###
    for user_n in range(1, n+1):

        # distance
        ax1.plot(X[method]["user_" + str(user_n)], Y[method]["user_" + str(user_n)], label="user_" + str(user_n))
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_ylim(-0.5, 0.5)
        ax1.title.set_text('Distance From User')
        if method == "GUI":
            region(ax1, 0.8, 1.1)
        elif method == "global":
            region(ax1, 0.5, 0.8)
        elif method == "local":
            region(ax1, 0.8, 1.1)



        # height
        ax2.plot(X[method]["user_" + str(user_n)], Z[method]["user_" + str(user_n)])
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Z [m]')
        ax2.set_ylim(0, 0.8)
        ax2.title.set_text('Height From Table')
        if method == "GUI":
            region(ax2, 0.2, 0.5)
        elif method == "global":
            region(ax2, 0.2, 0.5)
        elif method == "local":
            region(ax2, 0.5, 0.8)

        # orientation
        ax3.plot(X[method]["user_" + str(user_n)], Quat[method]["user_" + str(user_n)])
        ax3.set_xlabel('X [m]')
        ax3.set_ylabel('Quaternion')
        ax3.set_ylim(0, 1.5)
        ax3.title.set_text('End-Effector Orientation')
        if method == "GUI":
            region(ax3, 0.5, 0.8)
        elif method == "global":
            region(ax3, 0.8, 1.1)
        elif method == "local":
            region(ax3, 0.2, 0.5)


    plt.rc('axes', prop_cycle=(cycler('color', [(0.1, 0.2, 0.5), 'g', 'b', 'y'])))
    plt.tight_layout()
    plt.savefig("results_plot/" + method + ".png")
