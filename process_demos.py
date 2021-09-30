import pickle
import numpy as np
import argparse
import os
import sys


savename = "processed.pkl"

noise = 0.0
n_upsamples = 1
n_lookahead = 1

segment_ignore = int(sys.argv[1])

sapairs = []
for filename in os.listdir("demos"):
    traj = pickle.load(open("demos/" + filename, "rb"))
    print(filename, len(traj))
    traj = np.asarray(traj)
    for idx in range(len(traj) - n_lookahead):
        if traj[idx][6] == segment_ignore:
            continue
        s_base = traj[idx][0:6]
        sp = traj[idx + n_lookahead][0:6]
        for _ in range(n_upsamples):
            s = np.copy(s_base) + np.random.normal(0, noise, 6)
            a = sp - s
            sapairs.append(s.tolist() + a.tolist())

print(len(sapairs))
pickle.dump(sapairs, open("data/" + savename, "wb"))

# train in ee space