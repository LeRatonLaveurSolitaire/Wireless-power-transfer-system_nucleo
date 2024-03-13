"""Model of the neural network."""

import torch
from torch import nn


class NN_model(nn.Module):
    """Class of the model."""

    def __init__(self, input_size: int = None, output_size: int = None):
        """Class constructor.

        Args:
            input_size (int): Size of the input layer. Defaults to None.
            output_size (int): Size of the output layer. Defaults to None.
        """

        super(NN_model, self).__init__()

        self.input_size = input_size
        self.hidden1_size = 128
        self.hidden2_size = 128
        self.hidden3_size = 128
        self.hidden4_size = 128
        # self.hidden5_size = 256
        # self.hidden6_size = 256
        # self.hidden7_size = 100
        # self.hidden8_size = 100
        # self.hidden9_size = 100
        # self.hidden10_size = 100
        # self.hidden11_size = 100
        # self.hidden12_size = 100
        # self.hidden13_size = 100
        # self.hidden14_size = 100
        # self.hidden15_size = 100
        # self.hidden16_size = 100
        # self.hidden17_size = 100
        # self.hidden18_size = 100
        # self.hidden19_size = 100
        self.output_size = output_size

        self.linear1 = nn.Linear(self.input_size, self.hidden1_size)
        self.relu1 = nn.ReLU()
        self.linear2 = nn.Linear(self.hidden1_size, self.hidden2_size)
        self.relu2 = nn.ReLU()
        self.linear3 = nn.Linear(self.hidden2_size, self.hidden3_size)
        self.relu3 = nn.ReLU()
        self.linear4 = nn.Linear(self.hidden3_size, self.hidden4_size)
        self.relu4 = nn.ReLU()
        self.linear5 = nn.Linear(self.hidden4_size, self.output_size)
        # self.relu5 = nn.ReLU()
        # self.linear6 = nn.Linear(self.hidden5_size, self.hidden6_size)
        # self.relu6 = nn.ReLU()
        # self.linear7 = nn.Linear(self.hidden6_size, self.hidden7_size)
        # self.relu7 = nn.ReLU()
        # self.linear8 = nn.Linear(self.hidden7_size, self.hidden8_size)
        # self.relu8 = nn.ReLU()
        # self.linear9 = nn.Linear(self.hidden8_size, self.hidden9_size)
        # self.relu9 = nn.ReLU()
        # self.linear10 = nn.Linear(self.hidden9_size, self.hidden10_size)
        # self.relu10 = nn.ReLU()
        # self.linear11 = nn.Linear(self.hidden10_size, self.hidden11_size)
        # self.relu11 = nn.ReLU()
        # self.linear12 = nn.Linear(self.hidden11_size, self.hidden12_size)
        # self.relu12 = nn.ReLU()
        # self.linear13 = nn.Linear(self.hidden12_size, self.hidden13_size)
        # self.relu13 = nn.ReLU()
        # self.linear14 = nn.Linear(self.hidden13_size, self.hidden14_size)
        # self.relu14 = nn.ReLU()
        # self.linear15 = nn.Linear(self.hidden14_size, self.hidden15_size)
        # self.relu15 = nn.ReLU()
        # self.linear16 = nn.Linear(self.hidden15_size, self.hidden16_size)
        # self.relu16 = nn.ReLU()
        # self.linear17 = nn.Linear(self.hidden16_size, self.hidden17_size)
        # self.relu17 = nn.ReLU()
        # self.linear18 = nn.Linear(self.hidden17_size, self.hidden18_size)
        # self.relu18 = nn.ReLU()
        # self.linear19 = nn.Linear(self.hidden18_size, self.hidden19_size)
        # self.relu19 = nn.ReLU()
        # self.linear20 = nn.Linear(self.hidden19_size, self.output_size)

    def forward(self, x: torch.tensor = None) -> torch.tensor:
        """Forward pass function.

        Args:
            x (torch.tensor): input vector

        Returns:
            int: predicted number of nodes until the end
        """

        out = self.linear1(x)
        out = self.relu1(out)
        out = self.linear2(out)
        out = self.relu2(out)
        out = self.linear3(out)
        out = self.relu3(out)
        out = self.linear4(out)
        out = self.relu4(out)
        out = self.linear5(out)
        # out = self.relu5(out)
        # out = self.linear6(out)
        # out = self.relu6(out)
        # out = self.linear7(out)
        # out = self.relu7(out)
        # out = self.linear8(out)
        # out = self.relu8(out)
        # out = self.linear9(out)
        # out = self.relu9(out)
        # out = self.linear10(out)
        # out = self.relu10(out)
        # out = self.linear11(out)
        # out = self.relu11(out)
        # out = self.linear12(out)
        # out = self.relu12(out)
        # out = self.linear13(out)
        # out = self.relu13(out)
        # out = self.linear14(out)
        # out = self.relu14(out)
        # out = self.linear15(out)
        # out = self.relu15(out)
        # out = self.linear16(out)
        # out = self.relu16(out)
        # out = self.linear17(out)
        # out = self.relu17(out)
        # out = self.linear18(out)
        # out = self.relu18(out)
        # out = self.linear19(out)
        # out = self.relu19(out)
        # out = self.linear20(out)

        return out
