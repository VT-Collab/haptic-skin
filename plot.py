import time
import numpy as np
import pickle
import argparse
import matplotlib.pyplot as plt


# for all users

fig = plt.figure(figsize=(10,8))
# fig.suptitle('A tale of 2 subplots')

X = {}
Y = {}
Z = {}

for method in ["none", "GUI", "local", "global"]:
    filename = "data/demos/user_1_" + method +  ".pkl"
    file = open(filename, "rb")
    data = pickle.load(file)

    ee = data["ee positions"]
    points = np.array(ee).T

    p = data["pressure"]
    t = data["time"]

    X[method] = points[0,:]
    Y[method] = points[1,:]
    Z[method] = points[2,:]


### plot user demonstrations ###


# front projection: XZ
ax2 = fig.add_subplot(2, 2, 1)
plt.plot(X["none"], Z["none"])
plt.plot(X["GUI"], Z["GUI"])
plt.plot(X["local"], Z["local"])
plt.plot(X["global"], Z["global"])
plt.xlabel('X [m]')
plt.ylabel('Z [m]')

# side projection: YZ
ax3 = fig.add_subplot(2, 2, 2)
plt.plot(Y["none"], Z["none"])
plt.plot(Y["GUI"], Z["GUI"])
plt.plot(Y["local"], Z["local"])
plt.plot(Y["global"], Z["global"])

# top projection: XY
ax4 = fig.add_subplot(2, 2, 3)
l1 = plt.plot(X["none"], Y["none"])
l2 = plt.plot(X["GUI"], Y["GUI"])
l3 = plt.plot(X["local"], Y["local"])
l4 = plt.plot(X["global"], Y["global"])

lines_labels = [ax.get_legend_handles_labels() for ax in fig.axes]
lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
fig.legend(lines, labels, ncol=4, loc='upper center')

# plt.savefig("test.svg")

plt.show()
