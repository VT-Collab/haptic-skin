import pickle
import numpy as np
import argparse
import os


savename = "processed.pkl"

noise = 0.0
n_upsamples = 1
n_lookahead = 5

sapairs = []
for filename in os.listdir("demos"):
    traj = pickle.load(open("demos/" + filename, "rb"))
    print(filename, len(traj))
    traj = np.asarray(traj)
    for idx in range(len(traj) - n_lookahead):
        s_base = traj[idx]
        sp = traj[idx + n_lookahead]
        for _ in range(n_upsamples):
            s = np.copy(s_base) + np.random.normal(0, noise, 6)
            a = sp - s
            sapairs.append(s.tolist() + a.tolist())

print(len(sapairs))
pickle.dump(sapairs, open("data/" + savename, "wb"))
