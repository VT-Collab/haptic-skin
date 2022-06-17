import pickle
import numpy as np
import os
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
import csv

x_margin_1 = 0.15
x_margin_2 = 0.45
x_margin_3 = 0.75
y_margin = 0.15

R_desire = np.array([[ 7.44356863e-01,  6.66865793e-01,  3.49696414e-02],
                    [ 6.66166262e-01, -7.45177629e-01,  3.05419708e-02],
                    [ 4.64259900e-02,  5.61469737e-04, -9.98921575e-01]])
Q_desire = Quaternion(matrix=R_desire)


def process_run(filename):
    data = pickle.load(open("data/" + filename, "rb"))
    t = np.array(data["time"])
    q = np.array(data["joint positions"])
    xyz = np.array(data["ee positions"])
    rot = np.array(data["rotation matrix"])
    p = np.array(data["pressure"])
    return t, q, xyz, rot, p

def get_features(filename):
    t, q, xyz, rot, p = process_run(filename)
    segment = [None]*len(t)
    for timestep in range(len(t)):
        if xyz[timestep, 0] < x_margin_1:
            segment[timestep] = 1
        elif x_margin_1 <= xyz[timestep, 0] <= x_margin_2:
            segment[timestep] = 2
        elif xyz[timestep, 0] > x_margin_3 or xyz[timestep, 1] > y_margin:
            segment[timestep] = 4
        elif xyz[timestep, 0] > x_margin_2:
            segment[timestep] = 3
    height = xyz[:, 2]
    dist = np.abs(-0.5 - xyz[:, 1])
    align = [None]*len(t)
    for timestep in range(len(t)):
        Q_curr = Quaternion(matrix=rot[timestep])
        align[timestep] = Quaternion.absolute_distance(Q_desire, Q_curr)
    return np.array(segment), height, dist, np.array(align)

def segment_features(filename, method):
    s, h, d, a = get_features(filename)
    error = [0, 0, 0]
    counter = [0, 0, 0]
    # GUI: height, orientation, distance
    # Local: orientation, height, distance
    # Global: height, distance, orientation
    for timestep in range(len(s)):
        if method == "none":
            if s[timestep] == 1:
                error[0] += h[timestep]
                counter[0] += 1
            error[1] += d[timestep]
            counter[1] += 1
            error[2] += a[timestep]
            counter[2] += 1
        if s[timestep] == 1:
            if method == "GUI":
                error[0] += h[timestep]
                counter[0] += 1
            if method == "local":
                error[2] += a[timestep]
                counter[2] += 1
            if method == "global":
                error[0] += h[timestep]
                counter[0] += 1
        if s[timestep] == 2:
            if method == "GUI":
                error[2] += a[timestep]
                counter[2] += 1
            if method == "local":
                error[0] += h[timestep]
                counter[0] += 1
            if method == "global":
                error[1] += d[timestep]
                counter[1] += 1
        if s[timestep] == 3:
            if method == "GUI":
                error[1] += d[timestep]
                counter[1] += 1
            if method == "local":
                error[1] += d[timestep]
                counter[1] += 1
            if method == "global":
                error[2] += a[timestep]
                counter[2] += 1
    return np.array([error[0]/counter[0], error[1]/counter[1], error[2]/counter[2]])

def stop_time(filename, threshold):
    t, q, xyz, rot, p = process_run(filename)
    count = 0.0
    for timestep in range(1, len(t)):
        prev_joint = q[timestep-1,:]
        curr_joint = q[timestep,:]
        if np.linalg.norm(curr_joint - prev_joint) <= threshold:
            count += t[timestep] - t[timestep-1]
    return count

def improvement(user_number, method):
    filename = "user_" + str(user_number) + "_" + method + ".pkl"
    return segment_features(filename, method)

def plot_times():
    times_gui = []
    times_loc = []
    times_glo = []
    for filename in os.listdir("data"):
        t, q, xyz, rot, p = process_run(filename)
        if filename[-7] == "G":
            times_gui.append(t[-1])
        if filename[-7] == "c":
            times_loc.append(t[-1])
        if filename[-7] == "b":
            times_glo.append(t[-1])
    times = np.array([times_gui, times_loc, times_glo])
    times_mean = np.mean(times, axis=1)
    times_sem = np.std(times, axis=1) / np.sqrt(len(times_gui))
    with open('total_time.csv', 'w') as f:
        writer = csv.writer(f)
        for idx in range(len(times_gui)):
            writer.writerow([times_gui[idx], times_loc[idx], times_glo[idx]])
    plt.bar(range(3), times_mean, yerr=times_sem)
    plt.show()

def plot_stop_times(threshold=0.01):
    times_gui = []
    times_loc = []
    times_glo = []
    for filename in os.listdir("data"):
        time_stopped = stop_time(filename, threshold)
        if filename[-7] == "G":
            times_gui.append(time_stopped)
        if filename[-7] == "c":
            times_loc.append(time_stopped)
        if filename[-7] == "b":
            times_glo.append(time_stopped)
    times = np.array([times_gui, times_loc, times_glo])
    times_mean = np.mean(times, axis=1)
    times_sem = np.std(times, axis=1) / np.sqrt(len(times_gui))
    with open('stop_time.csv', 'w') as f:
        writer = csv.writer(f)
        for idx in range(len(times_gui)):
            writer.writerow([times_gui[idx], times_loc[idx], times_glo[idx]])
    plt.bar(range(3), times_mean, yerr=times_sem)
    plt.show()

def plot_improvement(users):
    improve_gui = []
    improve_loc = []
    improve_glo = []
    for idx in users:
        baseline = improvement(idx, "none")
        improve_gui.append(sum(baseline - improvement(idx, "GUI")))
        improve_loc.append(sum(baseline - improvement(idx, "local")))
        improve_glo.append(sum(baseline - improvement(idx, "global")))
    improve = np.array([improve_gui, improve_loc, improve_glo])
    improve_mean = np.mean(improve, axis=1)
    improve_sem = np.std(improve, axis=1) / np.sqrt(len(users))
    with open('improve.csv', 'w') as f:
        writer = csv.writer(f)
        for idx in range(len(improve_gui)):
            writer.writerow([improve_gui[idx], improve_loc[idx], improve_glo[idx]])
    plt.bar(range(3), improve_mean, yerr=improve_sem)
    plt.show()



plot_improvement([1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13])
plot_stop_times(0.001)
plot_times()



# def plot_traj(users):
#     for user_number in users:
#         filename1 = "user_" + str(user_number) + "_GUI.pkl"
#         filename2 = "user_" + str(user_number) + "_local.pkl"
#         filename3 = "user_" + str(user_number) + "_global.pkl"
#         _, _, xyz1, _, _ = process_run(filename1)
#         _, _, xyz2, _, _ = process_run(filename2)
#         _, _, xyz3, _, _ = process_run(filename3)
#         plt.plot(xyz1[:,0], xyz1[:,1], 'b-')
#         plt.plot(xyz2[:,0], xyz2[:,1], 'g-')
#         plt.plot(xyz3[:,0], xyz3[:,1], 'r-')
#     plt.show()

# plot_traj([1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13])
