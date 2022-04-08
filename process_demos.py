import pickle
import numpy as np
import os, sys


folder = "demos"

noise = 0.01        #hyperparameter
n_upsamples = 10
n_lookahead = 1


sapairs = []
for filename in os.listdir(folder):
    traj = pickle.load(open(folder + "/" + filename, 'rb'))
    print("I am loading file: ", folder + "/" + filename)
    print("it has this many data points: ", len(traj))
    traj = np.asarray(traj)
    for idx in range(len(traj) - n_lookahead):
        step = traj[idx]
        next_step = traj[idx + n_lookahead]
                
        # separate states from goal
        s_base = step[0]
        sp = next_step[0]

        for _ in range(n_upsamples):
            s = np.copy(s_base) + np.random.normal(0, noise, 6)
            a = sp - s
            # pair states, actions, and goal
            sapairs.append(s.tolist() + a.tolist() + step[1])

pickle.dump(sapairs, open("data/sa_pairs.pkl", "wb"))
print("I have this many state-action pairs: ", len(sapairs))
