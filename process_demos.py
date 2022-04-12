import pickle
import numpy as np
import os, sys
import argparse



parser = argparse.ArgumentParser(description='Preparing state-action pair dataset')
parser.add_argument('--who', help='expert vs. user(i)', type=str)
parser.add_argument('--feature', help='XY, Z, ROT', type=str)
args = parser.parse_args()


# hyperparameters
noise = 0.01        
n_upsamples = 10
n_lookahead = 1


folder = "data/demos/" + args.feature

if args.who == "expert":
    files = ['expert_1.pkl', 'expert_3.pkl', 'expert_2.pkl']
elif args.who[0:4] == "user":
    files = ['expert_1.pkl', 'expert_3.pkl', 'expert_2.pkl', args.who + '.pkl', args.who + '.pkl']


sapairs = []
for filename in files:
    traj = pickle.load(open(folder + "/" + filename, 'rb'))
    print("I am loading file: ", folder + "/" + filename)
    print("it has this many data points: ", len(traj))
    traj = np.asarray(traj)

    for idx in range(len(traj) - n_lookahead):
  
        s = traj[idx]
        s_next = traj[idx + n_lookahead]              

        for _ in range(n_upsamples):
            s = np.copy(s) + np.random.normal(0, noise, 6)
            a = s_next - s
            # pair states and actions
            sapairs.append(s.tolist() + a.tolist())

pickle.dump(sapairs, open("data/training/" + args.feature + "/" + args.who + "_sa_pairs.pkl", "wb"))
print("I have this many state-action pairs: ", len(sapairs))
