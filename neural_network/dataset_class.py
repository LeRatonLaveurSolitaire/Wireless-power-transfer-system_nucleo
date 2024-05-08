import pickle
from torch.utils.data import Dataset
import torch


class CustomDataset(Dataset):
    """Custor torch dataset with save method."""

    def __init__(self, data: list = None):
        """Class constructor.

        Args:
            data (list, optional): data constituing the dataset. Defaults to None.
                                    This list should be organise as the following :
                                    [[[input_values],[output_values]],
                                    [[input_values],[output_values]],
                                    [[input_values],[output_values]],
                                    [[input_values],[output_values]],
                                    ...
                                    ]
        """
        self.data = data

    def __len__(self) -> int:
        """Len method.

        Returns:
            int: length of the dataset.
        """

        return len(self.data)

    def __getitem__(self, idx: int = None) -> tuple:
        """Get item method.

        Args:
            idx (int): index of the item. Defaults to None.

        Returns:
            tuple: 2 tensors, 1 with the input values and the orther with the output value
        """
        input_values, output_values = self.data[idx]
        input_tensor = torch.tensor(input_values, dtype=torch.float)
        output_tensor = torch.tensor(output_values, dtype=torch.float)
        return input_tensor, output_tensor

    def save(self, file_path: str = None) -> None:
        """Function for saving a torch dataset in a pickle file.

        Args:
            file_path (str): file name or path to store the dataset. Defaults to None.

        Raises:
            Exception: in case of empty file_path
        """
        if not (file_path):
            raise Exception("Empty file path")

        with open(file_path, "wb") as f:
            pickle.dump(self, f)

    def load(self, file_path: str = None) -> Dataset:
        """Function for loading a torch dataset from a pickle file.

        Args:
            file_path (str): File path. Defaults to None.

        Raises:
            Exception: in case of empty file path.

        Returns:
            Dataset: PyTorch dataset loaded from the pickle file
        """
        if not (file_path):
            raise Exception("Empty file path")

        with open(file_path, "rb") as f:
            pickled = pickle.load(f)
        self.data = pickled.data
