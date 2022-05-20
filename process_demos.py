import pickle
import numpy as np
import argparse


parser = argparse.ArgumentParser(description='Preparing state-action pair dataset')
parser.add_argument('--who', help='expert vs. user(i)', type=str, default="expert")
parser.add_argument('--feature', help='XY, Z, ROT', type=str, default="XY")
args = parser.parse_args()


# hyperparameters
noise = 0.01
n_upsamples = 10
n_lookahead = 1


folder = "data/demos/" + args.feature

if args.who == "expert":
    files = ['expert_1.pkl', 'expert_3.pkl', 'expert_2.pkl']
elif args.who[0:4] == "user":
    files = ['expert_1.pkl', 'expert_3.pkl', 'expert_2.pkl',
            args.who + '.pkl', args.who + '.pkl', args.who + '.pkl']

sa_pairs = []
for filename in files:
    traj = pickle.load(open(folder + "/" + filename, 'rb'))
    print("[*] Loading file: ", folder + "/" + filename)
    print("[*] Number of data points: ", len(traj))
    traj = np.asarray(traj)
    for idx in range(len(traj) - n_lookahead):
        state = traj[idx][:7]
        state_next = traj[idx + n_lookahead][:7]
        for _ in range(n_upsamples):
            state = np.copy(state) + np.random.normal(0, noise, 7)
            action = state_next - state
            sa_pairs.append(state.tolist() + action.tolist())

pickle.dump(sa_pairs, open("data/training/" + args.feature + "/" + args.who + "_sa_pairs.pkl", "wb"))
print("I have this many state-action pairs: ", len(sa_pairs))
