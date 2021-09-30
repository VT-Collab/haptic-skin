import pickle
import numpy as np
import argparse
import os
import sys


parser = argparse.ArgumentParser(description='process demos for a specific task')
parser.add_argument('--task', type=int, default=0)
parser.add_argument('--segment', type=int, default=0)
args = parser.parse_args()

folder = "demos/task" + str(args.task)
savename = "processed.pkl"

noise = 0.0
n_upsamples = 1
n_lookahead = 1
n_excluded = 0

sapairs = []
for filename in os.listdir(folder):
    traj = pickle.load(open(folder + "/" + filename, "rb"))
    print(folder, filename, len(traj))
    traj = np.asarray(traj)
    for idx in range(len(traj) - n_lookahead):
        if traj[idx][6] == args.segment:
            n_excluded += 1
            continue
        s_base = traj[idx][0:6]
        sp = traj[idx + n_lookahead][0:6]
        for _ in range(n_upsamples):
            s = np.copy(s_base) + np.random.normal(0, noise, 6)
            a = sp - s
            sapairs.append(s.tolist() + a.tolist())

print("state-action pairs: ", len(sapairs), "excluded data: ", n_excluded)
pickle.dump(sapairs, open("data/" + savename, "wb"))