import torch
import pickle
import numpy as np
from train_model import MLP
import matplotlib.pyplot as plt



class Model(object):
    def __init__(self):
        self.model = MLP()
        model_dict = torch.load("models/MLP_model", map_location='cpu')
        self.model.load_state_dict(model_dict)
        self.model.eval

    def decoder(self, state):
        s_tensor = torch.FloatTensor(state)
        action = self.model.decoder(s_tensor).detach().numpy()
        return action


def main():

    model = Model()
    dataname = "demos/1.pkl"
    data = pickle.load(open(dataname, "rb"))

    uncertainty = []
    for state in data:
        actions = []
        for idx in range(100):
            actions.append(model.decoder(state))
        actions = np.asarray(actions)
        uncertainty.append(sum(np.std(actions, axis=0)))

    plt.plot(range(len(uncertainty)), uncertainty)
    plt.show()


if __name__ == "__main__":
    main()
