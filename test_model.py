import torch
import pickle
import numpy as np
from train_model import MLP
import matplotlib.pyplot as plt
import argparse


class Model(object):
    def __init__(self, task, segment):
        self.model = MLP()
        model_dict = torch.load("models/MLP_model_task" + str(task) + "_segment" + str(segment), map_location='cpu')
        self.model.load_state_dict(model_dict)
        self.model.eval

    def decoder(self, state):
        s_tensor = torch.FloatTensor(state)
        action = self.model.decoder(s_tensor).detach().numpy()
        return action


def main():

    parser = argparse.ArgumentParser(description='visualize where the model is uncertain')
    parser.add_argument('--task', type=int, default=0)
    parser.add_argument('--segment', type=int, default=0)
    parser.add_argument('--trial', type=int, default=0)
    args = parser.parse_args()
    model = Model(args.task, args.segment)

    dataname = "demos/task" + str(args.task) + "/trial" + str(args.trial) + ".pkl"
    data = pickle.load(open(dataname, "rb"))

    n_samples = 100

    uncertainty = []
    segment = []
    for state in data:
        actions = []
        for idx in range(n_samples):
            actions.append(model.decoder(state[:6]))
        actions = np.asarray(actions)
        uncertainty.append(sum(np.std(actions, axis=0)))
        segment.append(state[6])

    segment = np.asarray(segment) * max(uncertainty) / 2.0

    plt.plot(range(len(segment)), segment)
    plt.plot(range(len(uncertainty)), uncertainty)
    plt.show()


if __name__ == "__main__":
    main()
