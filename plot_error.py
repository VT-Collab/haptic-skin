import time
import numpy as np
import pickle
import argparse
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from scipy import interpolate
from utils import *


Panda = TrajectoryClient()

def average(lst):
    return sum(lst) / len(lst)

def region(ax, start_marg, end_marg):
    ax.axvspan(start_marg, end_marg, color='#a1d99b')

def get_idx(X):
    idx_1 = len(X[X <= x_margin_1])
    idx_2 = idx_1 + len(X[(X > x_margin_1) & (X <= x_margin_2)])
    idx_3 = idx_2 + len(X[(X > x_margin_2) & (X < x_margin_3)])
    return idx_1, idx_2, idx_3

def mean_error(arr, base):
    error = np.mean(abs(base - arr))
    return np.round(error, 3)

def quat_error(R):
    diff = []
    for r in R:
        diff.append(Quaternion.absolute_distance(Panda.rot2quat(R_desire),
        Panda.rot2quat(r)))
    return np.array(diff)

X = {}
Y = {}
Z = {}
Quat = {}
h_base = 0.0
dist_base = -0.45
orien_base = 0.0
n = 9

for method in ["GUI", "local", "global"]:
    x = {}
    y = {}
    z = {}
    quat = {}
    for user_n in range(1, n+1):
        filename = "data/demos/user_" + str(user_n) + "_" + method +  ".pkl"
        file = open(filename, "rb")
        data = pickle.load(file)

        R = data["rotation matrix"]
        ee = data["ee positions"]
        points = np.array(ee).T
        x["user_" + str(user_n)] = points[0,:]
        marg_0_start = np.where(x["user_" + str(user_n)] > -0.15)
        marg_0_idx = marg_0_start[0][0]

        marg_1_idx, marg_2_idx, marg_3_idx = get_idx(points[0, :])
        if method == "GUI":
            z["user_" + str(user_n)] = mean_error(points[2,:][marg_0_idx:marg_1_idx], h_base)
            quat["user_" + str(user_n)] = mean_error(quat_error(R[marg_1_idx:marg_2_idx]), orien_base)
            y["user_" + str(user_n)] = mean_error(points[1,:][marg_2_idx:marg_3_idx], dist_base)
        elif method == "global":
            z["user_" + str(user_n)] = mean_error(points[2,:][marg_0_idx:marg_1_idx], h_base)
            y["user_" + str(user_n)] = mean_error(points[1,:][marg_1_idx:marg_2_idx], dist_base)
            quat["user_" + str(user_n)] = mean_error(quat_error(R[marg_2_idx:marg_3_idx]), orien_base)
        elif method == "local":
            quat["user_" + str(user_n)] = mean_error(quat_error(R[marg_0_idx:marg_1_idx]), orien_base)
            z["user_" + str(user_n)] = mean_error(points[2,:][marg_1_idx:marg_2_idx], h_base)
            y["user_" + str(user_n)] = mean_error(points[1,:][marg_2_idx:marg_3_idx], dist_base)
    X[method] = x
    Y[method] = y
    Z[method] = z
    Quat[method] = quat


### user feature errors ###
GUI_h_error = Z["GUI"].values()
GUI_orien_error = Quat["GUI"].values()
GUI_dist_error = Y["GUI"].values()

global_h_error = Z["global"].values()
global_dist_error = Y["global"].values()
global_orien_error = Quat["global"].values()

local_orien_error = Quat["local"].values()
local_h_error = Z["local"].values()
local_dist_error = Y["local"].values()


### plot error bars ###
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(12,4))
fig.suptitle('\n')

# set width of bar
barWidth = 0.25

# Set position of bar on X axis
br1 = np.arange(n)
br2 = [x + barWidth for x in br1]
br3 = [x + barWidth for x in br2]

plt.setp((ax1, ax2, ax3),
        xticks=br1+barWidth, xticklabels=br1+1,
        xlabel='Users', ylabel='Percent Error')

# plot distance error
ax1.bar(br1, GUI_dist_error, color ='#d9d9d9', width = barWidth, label ='GUI')
ax1.bar(br2, local_dist_error, color ='#b3de69', width = barWidth, label ='Local')
ax1.bar(br3, global_dist_error, color ='#ff7f00', width = barWidth, label ='Global')
ax1.title.set_text('Distance From User')

# plot height error
ax2.bar(br1, GUI_h_error, color ='#d9d9d9', width = barWidth, label ='GUI')
ax2.bar(br2, local_h_error, color ='#b3de69', width = barWidth, label ='Local')
ax2.bar(br3, global_h_error, color ='#ff7f00', width = barWidth, label ='Global')
ax2.title.set_text('Height From Table')

# plot orientation error
ax3.bar(br1, GUI_orien_error, color ='#d9d9d9', width = barWidth, label ='GUI')
ax3.bar(br2, local_orien_error, color ='#b3de69', width = barWidth, label ='Local')
ax3.bar(br3, global_orien_error, color ='#ff7f00', width = barWidth, label ='Global')
ax3.title.set_text('End-Effector Orientation')

handles, labels = ax1.get_legend_handles_labels()
fig.legend(handles, labels, loc='upper center', ncol=3)

plt.tight_layout()
plt.savefig("results_plot/users_error.png")


### user mean feature errors ###
GUI_dist_error_mean = average(list(GUI_dist_error))
GUI_h_error_mean = average(list(GUI_h_error))
GUI_orien_error_mean = average(list(GUI_orien_error))
GUI_mean = [GUI_dist_error_mean, GUI_h_error_mean, GUI_orien_error_mean]

local_dist_error_mean = average(list(local_dist_error))
local_h_error_mean = average(list(local_h_error))
local_orien_error_mean = average(list(local_orien_error))
local_mean = [local_dist_error_mean, local_h_error_mean, local_orien_error_mean]

global_dist_error_mean = average(list(global_dist_error))
global_h_error_mean = average(list(global_h_error))
global_orien_error_mean = average(list(global_orien_error))
global_mean = [global_dist_error_mean, global_h_error_mean, global_orien_error_mean]



fig = plt.figure()
features = ['Distance','Height','Orient']

# Set position of bar on X axis
barWidth = 0.3
br1 = np.arange(len(features))
br2 = [x + barWidth for x in br1]
br3 = [x + barWidth for x in br2]

plt.bar(br1, GUI_mean, width = barWidth, color ='#d9d9d9', label = 'GUI')
plt.bar(br2, local_mean, width = barWidth, color ='#b3de69', label = 'Local')
plt.bar(br3, global_mean, width = barWidth, color ='#ff7f00', label = 'Global')

plt.xticks(br2, features)
plt.ylabel('Mean Percent Error')
plt.legend()
plt.savefig("results_plot/users_mean_error.png")
