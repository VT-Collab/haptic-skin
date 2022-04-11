import pickle
import numpy as np
import os, sys
import argparse



parser = argparse.ArgumentParser(description='Preparing state-action pair dataset')
parser.add_argument('--who', help='expert vs. user(i)', type=str)
parser.add_argument('--feature', help='XY, Z, ROT', type=str)
args = parser.parse_args()


#hyperparameters
noise = 0.01        
n_upsamples = 10
n_lookahead = 1


if args.who == "expert":
    files = ['expert_1.pkl', 'expert_3.pkl', 'expert_2.pkl']
elif args.who[0:4] == "user":
    print(args.who[0:4])
    exit()

folder = "data/demos/" + args.feature

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
            sapairs.append(s.tolist() + a.tolist())# + step[1])

pickle.dump(sapairs, open("training/" + args.set + "/sa_pairs.pkl", "wb"))
print("I have this many state-action pairs: ", len(sapairs))
