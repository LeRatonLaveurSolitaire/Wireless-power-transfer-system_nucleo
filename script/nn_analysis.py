import matplotlib.pyplot as plt
import numpy as np
import torch
from torch import tensor
import os
import random

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

sigma = 0.3

from nn_model import NN_model
import wpt_system_class as wpt

model_path = "script/model_noise_Z1.pt"


def delinearise_R_l(R_l: float = 0) -> float:
    """Recover R from the output data of the NN."""
    return 10 ** (R_l * 0.1)


def delinearise_M(M: float = 0) -> float:
    """Recover M from the output data of the NN."""
    L1 = 24e-6
    L2 = 24e-6
    return 10 ** ((M * 0.1)) * (0.1 * (L1 * L2) ** 0.5)


def nn_input_tensor(
    sys_impedance: list = None,
    sys_frequencies: list = None,
    L1: float = None,
    C1: float = None,
    R1: float = None,
) -> tensor:
    """Create the input_tensor from the computed impedance

    Args:
        sys_impedance (list): Impedance of the system. Defaults to None.
        sys_frequencies (list): Frequencies of the impedance. Defaults to None.
        L1 (float): Primary coil inductance in Henri. Defaults to None.
        R1 (float): Primary coil ESR in ohm. Defaults to None.
        C1 (float): Primary side series compensation capacitor in Farad. Defaults to None.
    Returns:
        tensor: input tensor for the neural network
    """

    wanted_frequencies = np.geomspace(50000, 144500, num=15, dtype=np.int64)

    input_tensor = []

    for i, frequency in enumerate(wanted_frequencies):
        index = np.argmin(
            np.array(
                [abs(sys_frequency - frequency) for sys_frequency in sys_frequencies]
            )
        )
        Z_estim = sys_impedance[index]
        Z1 = (
            1j * 2 * np.pi * frequency * L1 + R1 + 1 / (2 * np.pi * frequency * 1j * C1)
        )
        Z2 = Z_estim - Z1
        input_tensor.append(np.absolute(Z2) * random.gauss(mu=1, sigma=sigma))
        input_tensor.append(np.angle(Z2) * random.gauss(mu=1, sigma=sigma))

    return tensor(input_tensor, dtype=torch.float32)


def main():

    L1 = 24 * 1e-6
    C1 = 146e-9  # 1 / ((2 * np.pi * f0) ** 2 * L1)
    R1 = 0.075

    L2 = 24 * 1e-6
    C2 = 146e-9  # 1 / ((2 * np.pi * f0) ** 2 * L2)
    R2 = 0.075

    M_bound = [0.05 * (L1 * L2) ** (1 / 2), 0.50 * (L1 * L2) ** (1 / 2)]
    R_bound = [0.5, 5]

    nbr_points = 25

    result = {}

    neural_network = NN_model(input_size=30, output_size=2)
    checkpoint = torch.load(model_path)
    neural_network.load_state_dict(checkpoint)

    R_values = list(np.linspace(*R_bound, nbr_points))
    M_values = list(np.linspace(*M_bound, nbr_points))
    for R in R_values:
        for M in M_values:

            primary_s = wpt.transmitter(L=L1, C_s=C1, R=R1)
            secondary_s = wpt.reciever(L=L2, C_s=C2, R=R2, R_l=R)
            wpt_system = wpt.total_system(
                transmitter=primary_s, reciever=secondary_s, M=M, name="model"
            )

            sys_frequencies = np.geomspace(50000, 144500, num=15, dtype=np.int64)

            model_impedance = np.array(
                [wpt_system.impedance(freq) for freq in sys_frequencies]
            )

            input_tensor = nn_input_tensor(
                sys_impedance=model_impedance,
                sys_frequencies=sys_frequencies,
                L1=L1,
                C1=C1,
                R1=R1,
            )

            # Compute NN inferance

            with torch.inference_mode():
                output_tensor = neural_network(input_tensor)

            R_computed = delinearise_R_l(output_tensor[0].item())
            M_computed = delinearise_M(output_tensor[1].item())

            err = (abs(R - R_computed) / R + abs(M - M_computed) / M) / 2
            result[(R, M)] = [R_computed, M_computed, err]

    # Create a matrix to hold the values
    heatmap_data = np.zeros((len(R_values), len(M_values)))

    # Fill the matrix with data from the dictionary
    for R, M in result:
        heatmap_data[R_values.index(R), M_values.index(M)] = result[(R, M)][2] * 100

    # Create the heatmap
    plt.imshow(heatmap_data, cmap="RdYlGn_r", interpolation="nearest")

    # Add color bar
    plt.colorbar()

    # Set labels
    plt.xlabel("M (in µH)")
    plt.ylabel("R (in Ohm)")

    # Set ticks
    plt.xticks(
        np.arange(len(M_values)),
        list(np.array(np.array(M_values) * 1e6, dtype=np.int64)),
    )
    plt.yticks(
        np.arange(len(R_values)), np.array(np.array(R_values) * 10, dtype=np.int64) / 10
    )

    # Show plot
    plt.title(f"Heatmap of the estimation error ($sigma = {sigma})")
    plt.show()


if __name__ == "__main__":
    main()
