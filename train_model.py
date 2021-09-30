import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from torch.distributions import Normal
import torch.optim as optim
import pickle
import numpy as np
import argparse
import os
import sys


class MotionData(Dataset):

    def __init__(self, filename):
        self.data = pickle.load(open(filename, "rb"))

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return torch.FloatTensor(self.data[idx])


class MLP(nn.Module):

    def __init__(self):
        super(MLP, self).__init__()

        self.name = "MLP"
        self.d = 0.1

        self.mlp = nn.Sequential(
            nn.Linear(6, 30),
            nn.Tanh(),
            nn.Dropout(self.d),
            nn.Linear(30, 30),
            nn.Tanh(),
            nn.Dropout(self.d),
            nn.Linear(30, 6)
        )

        self.loss_func = nn.MSELoss()

    def decoder(self, s):
        return self.mlp(s)

    def forward(self, x):
        s = x[:, 0:6]
        a_target = x[:, 6:12]
        a_decoded = self.decoder(s)
        loss = self.loss(a_decoded, a_target)
        return loss

    def loss(self, a_decoded, a_target):
        return self.loss_func(a_decoded, a_target)


def main():

    parser = argparse.ArgumentParser(description='process demos and train model for a specific task')
    parser.add_argument('--task', type=int, default=0)
    parser.add_argument('--segment', type=int, default=0)
    args = parser.parse_args()

    folder = "demos/task" + str(args.task)
    savename = "processed.pkl"

    noise = 0.0
    n_upsamples = 1
    n_lookahead = 1
    n_excluded = 0

    sapairs = []
    for filename in os.listdir(folder):
        traj = pickle.load(open(folder + "/" + filename, "rb"))
        print(folder, filename, len(traj))
        traj = np.asarray(traj)
        for idx in range(len(traj) - n_lookahead):
            if traj[idx][6] == args.segment - 1:
                n_excluded += 1
                continue
            s_base = traj[idx][0:6]
            sp = traj[idx + n_lookahead][0:6]
            for _ in range(n_upsamples):
                s = np.copy(s_base) + np.random.normal(0, noise, 6)
                a = sp - s
                sapairs.append(s.tolist() + a.tolist())

    print("state-action pairs: ", len(sapairs), "excluded data: ", n_excluded)
    pickle.dump(sapairs, open("data/" + savename, "wb"))

    model = MLP()
    dataname = "data/processed.pkl"
    savename = "models/MLP_model_task" + str(args.task) + "_segment" + str(args.segment)
    data = pickle.load(open(dataname, "rb"))

    EPOCH = 1000
    BATCH_SIZE_TRAIN = int(len(data) / 10.0)
    LR = 0.01
    LR_STEP_SIZE = 360
    LR_GAMMA = 0.1

    train_data = MotionData(dataname)
    train_set = DataLoader(dataset=train_data, batch_size=BATCH_SIZE_TRAIN, shuffle=True)

    optimizer = optim.Adam(model.parameters(), lr=LR)
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=LR_STEP_SIZE, gamma=LR_GAMMA)

    for epoch in range(EPOCH):
        for batch, x in enumerate(train_set):
            optimizer.zero_grad()
            loss = model(x)
            loss.backward()
            optimizer.step()
        scheduler.step()
        print(epoch, loss.item())
    torch.save(model.state_dict(), savename)


if __name__ == "__main__":
    main()
