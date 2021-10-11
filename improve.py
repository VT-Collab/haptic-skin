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


SEGMENT ={(1,1,'gui'):1,(1,2,'gui'):1,(1,3,'gui'):3,
    (1,1,'table'):1,(1,2,'table'):3,(1,3,'table'):1,
    (1,1,'robot'):3,(1,2,'robot'):1,(1,3,'robot'):1,

    (2,1,'gui'):1,(2,2,'gui'):3,(2,3,'gui'):1,
    (2,1,'table'):3,(2,2,'table'):3,(2,3,'table'):1,
    (2,1,'robot'):1,(2,2,'robot'):1,(2,3,'robot'):3,

    (3,1,'gui'):1,(3,2,'gui'):3,(3,3,'gui'):1,
    (3,1,'table'):3,(3,2,'table'):1,(3,3,'table'):3,
    (3,1,'robot'):3,(3,2,'robot'):3,(3,3,'robot'):1,

    (4,1,'gui'):1,(4,2,'gui'):1,(4,3,'gui'):3,
    (4,1,'table'):1,(4,2,'table'):3,(4,3,'table'):1,
    (4,1,'robot'):3,(4,2,'robot'):1,(4,3,'robot'):1,

    (5,1,'gui'):1,(5,2,'gui'):3,(5,3,'gui'):1,
    (5,1,'table'):3,(5,2,'table'):3,(5,3,'table'):1,
    (5,1,'robot'):1,(5,2,'robot'):1,(5,3,'robot'):3,

    (6,1,'gui'):1,(6,2,'gui'):3,(6,3,'gui'):1,
    (6,1,'table'):3,(6,2,'table'):1,(6,3,'table'):3,
    (6,1,'robot'):3,(6,2,'robot'):3,(6,3,'robot'):1,

    (7,1,'gui'):1,(7,2,'gui'):1,(7,3,'gui'):3,
    (7,1,'table'):1,(7,2,'table'):3,(7,3,'table'):1,
    (7,1,'robot'):3,(7,2,'robot'):1,(7,3,'robot'):1,

    (8,1,'gui'):1,(8,2,'gui'):3,(8,3,'gui'):1,
    (8,1,'table'):3,(8,2,'table'):3,(8,3,'table'):1,
    (8,1,'robot'):1,(8,2,'robot'):1,(8,3,'robot'):3,

    (9,1,'gui'):1,(9,2,'gui'):3,(9,3,'gui'):1,
    (9,1,'table'):3,(9,2,'table'):1,(9,3,'table'):3,
    (9,1,'robot'):3,(9,2,'robot'):3,(9,3,'robot'):1,

    (10,1,'gui'):1,(10,2,'gui'):1,(10,3,'gui'):3,
    (10,1,'table'):1,(10,2,'table'):3,(10,3,'table'):1,
    (10,1,'robot'):3,(10,2,'robot'):1,(10,3,'robot'):1,}


class MotionData(Dataset):

    def __init__(self, data):
        self.data = data

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return torch.FloatTensor(self.data[idx])


class MLP(nn.Module):

    def __init__(self):
        super(MLP, self).__init__()

        self.name = "MLP"
        self.d = 0.2

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


    trial = 2
    method = "table"

    for task in range(1, 4):
        for user in range(4, 14):

            segment = SEGMENT[(user-3, task, method)]
            folder = "demos/task" + str(task)

            noise = 0.01
            n_upsamples = 10
            n_lookahead = 1
            n_excluded = 0

            sapairs = []
            for filename in os.listdir(folder):
                traj = pickle.load(open(folder + "/" + filename, "rb"))
                traj = np.asarray(traj)
                for idx in range(len(traj) - n_lookahead):
                    if traj[idx][6] == segment - 1:
                        n_excluded += 1
                        continue
                    s_base = traj[idx][0:6]
                    sp = traj[idx + n_lookahead][0:6]
                    for _ in range(n_upsamples):
                        s = np.copy(s_base) + np.random.normal(0, noise, 6)
                        a = sp - s
                        sapairs.append(s.tolist() + a.tolist())
                print(folder, filename, len(traj), n_excluded)

            print("state-action pairs: ", len(sapairs), "excluded segment: ", segment)

            newdata = []
            file = "demos/user" + str(user) + "/" + method + "/task" + str(task) + "_trial" + str(trial) + ".pkl"
            with open(file, "rb") as f:
                traj = pickle.load(f, encoding="latin1")
                for idx in range(len(traj) - n_lookahead):
                    s_base = traj[idx][0][0:6]
                    sp = traj[idx + n_lookahead][0][0:6]
                    for _ in range(n_upsamples):
                        s = np.copy(s_base) + np.random.normal(0, noise, 6)
                        a = sp - s
                        newdata.append(s.tolist() + a.tolist())

            sapairs += newdata

            print("number of new datapoints: ", len(newdata), "full data: ", len(sapairs))



            ### train model from dataset

            model = MLP()
            savename = "models/user" + str(user) + "_" + method + "_task" + str(task)
            data = sapairs

            EPOCH = 1000
            BATCH_SIZE_TRAIN = int(len(data) / 10.0)
            LR = 0.01
            LR_STEP_SIZE = 360
            LR_GAMMA = 0.1

            train_data = MotionData(data)
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
                if epoch % 100 == 0:
                    print(epoch, loss.item())
            torch.save(model.state_dict(), savename)


if __name__ == "__main__":
    main()
