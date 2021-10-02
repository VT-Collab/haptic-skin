import torch
import pickle
import numpy as np
from train_model import MLP
import matplotlib.pyplot as plt
import argparse
import os


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
    args = parser.parse_args()
    model = Model(args.task, args.segment)

    n_samples = 100
    noise = 0.01

    folder = "demos/task" + str(args.task)
    for filename in os.listdir(folder):
        data = pickle.load(open(folder + "/" + filename, "rb"))
        print(folder, filename)
        uncertainty = []
        segment = []
        for state in data:
            s = np.asarray(state[:6]) +  np.random.normal(0, noise, 6)
            actions = []
            for idx in range(n_samples):
                actions.append(model.decoder(s.tolist()))
            actions = np.asarray(actions)
            uncertainty.append(sum(np.std(actions, axis=0)))
            segment.append(state[6])
        segment = np.asarray(segment) * max(uncertainty) / 2.0
        plt.plot(range(len(uncertainty)), uncertainty)
    plt.title("task " + str(args.task) + " segment " + str(args.segment))
    plt.show()


if __name__ == "__main__":
    main()
