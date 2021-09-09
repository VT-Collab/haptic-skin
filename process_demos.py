import pickle
import numpy as np
import argparse


filename = "file-of-saved-demonstrations.pkl"
savename = "file-of-upsampled-state-action-pairs.pkl"

noise = 0.1
n_upsamples = 10

data = pickle.load(open("data/" + filename, "rb"))
sapairs = []
for traj in data:
    traj = np.asarray(traj)
    for idx in range(len(traj) - 1):
        s_base = traj[idx]
        sp = traj[idx + 1]
        for _ in range(n_upsamples):
            s = np.copy(s_base) + np.random.normal(0, noise, 7)
            a = sp - s
            sapairs.append(s.tolist() + a.tolist())

print(sapairs)
print(len(sapairs))
pickle.dump(sapairs, open("data/" + savename, "wb"))
