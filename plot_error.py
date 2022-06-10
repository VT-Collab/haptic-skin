import time
import numpy as np
import pickle
import argparse
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from scipy import interpolate

from utils import *


Panda = TrajectoryClient()

def region(ax, start_marg, end_marg):
    ax.axvspan(start_marg, end_marg, color='#a1d99b')


X = {}
Y = {}
Z = {}
Quat = {}
n = 6

for method in ["GUI", "local", "global"]:
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(12,4))
    fig.suptitle(method.upper())

    x = {}
    y = {}
    z = {}
    quat = {}


    def get_idx(X):
        idx_1 = len(X[X < x_margin_1])
        idx_2 = len(X[(X > x_margin_1) & (X < x_margin_2)])
        idx_3 = len(X[(X > x_margin_2) & (X < x_margin_3)])
        return idx_1, idx_2, idx_3


    for user_n in range(1, n+1):
        filename = "data/demos/user_" + str(user_n) + "_" + method +  ".pkl"
        file = open(filename, "rb")
        data = pickle.load(file)

        R = data["rotation matrix"]
        ee = data["ee positions"]
        points = np.array(ee).T
        x["user_" + str(user_n)] = points[0,:]



        marg_1_idx, marg_2_idx, marg_3_idx = get_idx(points[0, :])
        if method == "GUI":
            z["user_" + str(user_n)] = points[2,:][:marg_1_idx]
            y["user_" + str(user_n)] = points[1,:][marg_1_idx:marg_2_idx]
            R = R[marg_2_idx:marg_3_idx]

        elif method == "local":
            z["user_" + str(user_n)] = points[2,:][:marg_1_idx]
            R = R[:marg_1_idx]
            y["user_" + str(user_n)] = points[1,:][marg_1_idx:marg_2_idx]


        elif method == "global":
            pass


        quat_diff = []
        for r in R:
            quat_diff.append(Quaternion.absolute_distance(Panda.rot2quat(R_desire),
                                                          Panda.rot2quat(r)))
        quat["user_" + str(user_n)] = quat_diff


    X[method] = x
    Y[method] = y
    Z[method] = z
    Quat[method] = quat



    # ### plot user demonstrations ###
    # for user_n in range(1, n+1):
    #
    #     # distance
    #     ax1.plot(X[method]["user_" + str(user_n)], Y[method]["user_" + str(user_n)], label="user_" + str(user_n))
    #     ax1.set_xlabel('X [m]')
    #     ax1.set_ylabel('Y [m]')
    #     ax1.set_ylim(-0.5, 0.5)
    #     ax1.title.set_text('Distance From User')
    #     if method == "GUI":
    #         region(ax3, 0.8, 1.1)
    #     elif method == "global":
    #         region(ax3, 0.5, 0.8)
    #     elif method == "local":
    #         region(ax3, 0.8, 1.1)
    #
    #     # height
    #     ax2.plot(X[method]["user_" + str(user_n)], Z[method]["user_" + str(user_n)])
    #     ax2.set_xlabel('X [m]')
    #     ax2.set_ylabel('Z [m]')
    #     ax2.set_ylim(0, 0.8)
    #     ax2.title.set_text('Height From Table')
    #     if method == "GUI":
    #         region(ax1, 0.2, 0.5)
    #     elif method == "global":
    #         region(ax1, 0.2, 0.5)
    #     elif method == "local":
    #         region(ax1, 0.5, 0.8)
    #
    #     # orientation
    #     ax3.plot(X[method]["user_" + str(user_n)], Quat[method]["user_" + str(user_n)])
    #     ax3.set_xlabel('X [m]')
    #     ax3.set_ylabel('Quaternion')
    #     ax3.set_ylim(0, 1.5)
    #     ax3.title.set_text('End-Effector Orientation')
    #     if method == "GUI":
    #         region(ax2, 0.5, 0.8)
    #     elif method == "global":
    #         region(ax2, 0.8, 1.1)
    #     elif method == "local":
    #         region(ax2, 0.2, 0.5)
    #
    #
    # plt.tight_layout()
    # plt.savefig("results_plot/" + method + ".png")



# def compute_error(X, Y, Z, Quat, method, user):
#     X[method][user]
