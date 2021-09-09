import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import torch.optim as optim
import pickle
import numpy as np


class MotionData(Dataset):

    def __init__(self, filename):
        self.data = pickle.load(open(filename, "rb"))

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return torch.FloatTensor(self.data[idx])


class MLP(nn.Module):

    def __init__(self, latent_size):
        super(MLP, self).__init__()

        self.name = "MLP"
        self.fc1 = nn.Linear(7, 30)
        self.fc2 = nn.Linear(30, 30)
        self.fc3 = nn.Linear(30, 7)

        self.loss_func = nn.MSELoss()

    def decoder(self, s):
        h1 = torch.tanh(self.fc1(s))
        h2 = torch.tanh(self.fc2(h1))
        return self.fc3(h2)

    def forward(self, x):
        s = x[:, 0:7]
        a_target = x[:, 7:14]
        a_decoded = self.decoder(s)
        loss = self.loss(a_decoded, a_target)
        return loss

    def loss(self, a_decoded, a_target):
        return self.loss_func(a_decoded, a_target)


def main():

    model = MLP()
    dataname = "data/file-of-upsampled-state-action-pairs.pkl"
    savename = "models/MLP_model"
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
