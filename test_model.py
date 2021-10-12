import torch
import pickle
import numpy as np
from train_model import MLP
import matplotlib.pyplot as plt
import argparse
import os


class Model(object):
    def __init__(self, modelname):
        self.model = MLP()
        model_dict = torch.load(modelname, map_location='cpu')
        self.model.load_state_dict(model_dict)
        self.model.eval

    def decoder(self, state):
        s_tensor = torch.FloatTensor(state)
        action = self.model.decoder(s_tensor).detach().numpy()
        return action

def process(method, n_samples=100):
    U = []
    for task in range(1, 4):
        for user in range(4, 14):
            modelname = "models/user" + str(user) + "_" + method + "_task" + str(task)
            model = Model(modelname)
            file = "demos/user" + str(user) + "/" + method + "/task" + str(task) + "_trial1.pkl"
            uncertainty = []
            with open(file, "rb") as f:
                traj = pickle.load(f, encoding="latin1")
                print(modelname, file)
                for state in traj:
                    s = state[0]
                    actions = []
                    for idx in range(n_samples):
                        actions.append(model.decoder(s.tolist()))
                    actions = np.asarray(actions)
                    uncertainty.append(sum(np.std(actions, axis=0)))
            U.append(sum(uncertainty))
    return U


def main():

    u_gui = process("gui")
    u_table = process("table")
    u_robot = process("robot")
    data = [u_gui, u_table, u_robot]
    pickle.dump(data, open("uncertainty.pkl", "wb"))


if __name__ == "__main__":
    main()
