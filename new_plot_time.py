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
    error = np.mean(abs(base - arr) / len(arr))
    return np.round(100 * error, 3)

def quat_error(R):
    diff = []
    for r in R:
        diff.append(Quaternion.absolute_distance(Panda.rot2quat(R_desire), Panda.rot2quat(r)))
    return np.array(diff)

def compute_time(joint_pos, time_steps):
    t_active = 0
    seg_joint_pos = np.array(joint_pos)
    norm_pos = np.linalg.norm(seg_joint_pos, axis=1, keepdims=True)

    count = 0
    for idx in range(1, len(seg_joint_pos)):
        if np.linalg.norm(seg_joint_pos[idx] - seg_joint_pos[idx-1]) >= 0.01:
            count += 1
    return 100 * count/time_steps


X = {}
Y = {}
Z = {}
Quat = {}
n = len(USERS)

for method in ["GUI", "local", "global"]:
    x = {}
    t_y = {}
    t_z = {}
    t_quat = {}

    for user_n in USERS:
        filename = "data/demos/user_" + str(user_n) + "_" + method +  ".pkl"
        file = open(filename, "rb")
        data = pickle.load(file)
        print(data.keys())

        joint_pos = data["joint positions"]
        R = data["rotation matrix"]
        time = data["time"]
        ee = data["ee positions"]

        points = np.array(ee).T
        time_steps = len(time)
        x["user_" + str(user_n)] = points[0,:]

        marg_1_idx, marg_2_idx, marg_3_idx = get_idx(points[0, :])
        if method == "GUI":
            t_z["user_" + str(user_n)] = compute_time(joint_pos[:marg_1_idx], time_steps)
            t_quat["user_" + str(user_n)] = compute_time(joint_pos[marg_1_idx:marg_2_idx], time_steps)
            t_y["user_" + str(user_n)] = compute_time(joint_pos[marg_2_idx:marg_3_idx], time_steps)
        elif method == "global":
            t_z["user_" + str(user_n)] = compute_time(joint_pos[:marg_1_idx], time_steps)
            t_y["user_" + str(user_n)] = compute_time(joint_pos[marg_1_idx:marg_2_idx], time_steps)
            t_quat["user_" + str(user_n)] = compute_time(joint_pos[marg_2_idx:marg_3_idx], time_steps)
        elif method == "local":
            t_quat["user_" + str(user_n)] = compute_time(joint_pos[:marg_1_idx], time_steps)
            t_z["user_" + str(user_n)] = compute_time(joint_pos[marg_1_idx:marg_2_idx], time_steps)
            t_y["user_" + str(user_n)] = compute_time(joint_pos[marg_2_idx:marg_3_idx], time_steps)

    X[method] = x
    Y[method] = t_y
    Z[method] = t_z
    Quat[method] = t_quat

### user feature times ###
GUI_h_time = Z["GUI"].values()
GUI_orien_time = Quat["GUI"].values()
GUI_dist_time = Y["GUI"].values()

global_h_time = Z["global"].values()
global_dist_time = Y["global"].values()
global_orien_time = Quat["global"].values()

local_orien_time = Quat["local"].values()
local_h_time = Z["local"].values()
local_dist_time = Y["local"].values()


### plot time bars ###
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
        xlabel='Users', ylabel='Percent Time')

# plot distance time
ax1.bar(br1, GUI_dist_time, color ='#d9d9d9', width = barWidth, label ='GUI')
ax1.bar(br2, local_dist_time, color ='#b3de69', width = barWidth, label ='Local')
ax1.bar(br3, global_dist_time, color ='#ff7f00', width = barWidth, label ='Global')
ax1.title.set_text('Distance From User')

# plot height time
ax2.bar(br1, GUI_h_time, color ='#d9d9d9', width = barWidth, label ='GUI')
ax2.bar(br2, local_h_time, color ='#b3de69', width = barWidth, label ='Local')
ax2.bar(br3, global_h_time, color ='#ff7f00', width = barWidth, label ='Global')
ax2.title.set_text('Height From Table')

# plot orientation time
ax3.bar(br1, GUI_orien_time, color ='#d9d9d9', width = barWidth, label ='GUI')
ax3.bar(br2, local_orien_time, color ='#b3de69', width = barWidth, label ='Local')
ax3.bar(br3, global_orien_time, color ='#ff7f00', width = barWidth, label ='Global')
ax3.title.set_text('End-Effector Orientation')

handles, labels = ax1.get_legend_handles_labels()
fig.legend(handles, labels, loc='upper center', ncol=3)

plt.tight_layout()
plt.savefig("results_plot/users_time.png")


### user mean feature times ###
GUI_dist_time_mean = average(list(GUI_dist_time))
GUI_h_time_mean = average(list(GUI_h_time))
GUI_orien_time_mean = average(list(GUI_orien_time))
GUI_mean = [GUI_dist_time_mean, GUI_h_time_mean, GUI_orien_time_mean]

local_dist_time_mean = average(list(local_dist_time))
local_h_time_mean = average(list(local_h_time))
local_orien_time_mean = average(list(local_orien_time))
local_mean = [local_dist_time_mean, local_h_time_mean, local_orien_time_mean]

global_dist_time_mean = average(list(global_dist_time))
global_h_time_mean = average(list(global_h_time))
global_orien_time_mean = average(list(global_orien_time))
global_mean = [global_dist_time_mean, global_h_time_mean, global_orien_time_mean]

stacked_means = np.sum(np.array([GUI_mean, local_mean, global_mean]), axis=1, keepdims=True)
# exit()

fig = plt.figure()
features = ['Distance','Height','Orient', 'Overall']

# Set position of bar on X axis
barWidth = 0.25
br1 = np.arange(len(features))
br2 = [x + barWidth for x in br1]
br3 = [x + barWidth for x in br2]

plt.bar(br1, GUI_mean + list(stacked_means[0]), width = barWidth, color ='#d9d9d9', label = 'GUI')
plt.bar(br2, local_mean + list(stacked_means[1]), width = barWidth, color ='#b3de69', label = 'Local')
plt.bar(br3, global_mean + list(stacked_means[2]), width = barWidth, color ='#ff7f00', label = 'Global')

plt.xticks(br2, features)
plt.ylabel('Percent Time')
plt.legend(ncol=3, bbox_to_anchor=(0.62, 1.1))
plt.savefig("results_plot/users_mean_time.png")
