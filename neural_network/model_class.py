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

        return out
