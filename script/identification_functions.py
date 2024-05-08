"""
Usefull functions of the identification technique.
Enable the creation of the tensor from the raw signals.
"""

import numpy as np
from torch import tensor
import torch
import os

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"


def delinearise_R_l(R_l: float = 0) -> float:
    """Recover R from the output data of the NN."""
    return 10 ** (R_l * 0.1)


def delinearise_M(M: float = 0) -> float:
    """Recover M from the output data of the NN."""
    L1 = 24e-6
    L2 = 24e-6
    M_delin = 10 ** ((M * 0.1)) * (0.1 * (L1 * L2) ** 0.5)

    return M_delin


def fractional_decade_smoothing_impedance(
    impedances: np.array = None,
    frequencies: np.array = None,
    fractional_factor: int = None,
) -> np.array:
    """Fractional decade smoothing.

    Args:
        impedances (array): complex impedance values
        frequencies (array): frequency corresponding to the impedance
        fractional_factor (int): smoothing factor

    Returns:
        array: smoothed impedance after filtering
    """
    smoothed_impedances = []
    for i, freq in enumerate(frequencies):
        lower_bound = freq / fractional_factor
        upper_bound = freq * fractional_factor

        # Find indices within the frequency range for smoothing
        indices = np.where((frequencies >= lower_bound) & (frequencies <= upper_bound))[
            0
        ]

        # Calculate the smoothed impedance within the range
        smoothed_impedance = np.mean(np.absolute(impedances[indices])) * np.exp(
            1j * np.mean(np.angle(impedances[indices]))
        )
        smoothed_impedances.append(smoothed_impedance)

    return np.array(smoothed_impedances)


def extract_impedance(
    noisy_current: list = None,
    noisy_voltage: list = None,
    sampling_period: float = None,
    tau: float = None,
) -> tuple:
    """Extract the system impedance using the combination of spectral substraction (Gain) and simple impedance computation (Phase).

    Args:
        clean_current (list): clean current. Defaults to None.
        noisy_current (list): currrent with PRBS injection. Defaults to None.
        noisy_voltage (list): voltage with PRBS injection. Defaults to None.
        sampling_period (float): The delay between the signals, used for phase correction
        tau (float): The delay between the signals, used for phase correction. Defaults to None.
    """
    noisy_fft_current = np.fft.rfft(noisy_current)
    noisy_fft_voltage = np.fft.rfft(noisy_voltage)
    sys_frequencies = np.fft.rfftfreq(
        n=len(noisy_current),
        d=sampling_period,
    )

    system_impedance = noisy_fft_voltage / noisy_fft_current

    system_impedance = np.array(
        [
            system_impedance[i]
            * np.exp(-1j * np.pi * 2 * sys_frequencies[i] * tau + 1j * np.pi)
            for i in range(len(system_impedance))
        ]
    )

    return (
        system_impedance[1:],
        sys_frequencies[1:],
    )  # [1:] is used to remove the 0Hz to avoid division by 0


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
        # print(f"{index}\t{np.absolute(Z_estim)}\t{np.angle(Z_estim)}")
        Z1 = (
            1j * 2 * np.pi * frequency * L1 + R1 + 1 / (2 * np.pi * frequency * 1j * C1)
        )
        Z2 = Z_estim  # - Z1
        input_tensor.append(np.absolute(Z2))
        input_tensor.append(np.angle(Z2))

    return tensor(input_tensor, dtype=torch.float32)


def pretty_print(
    real_r: float = None,
    real_m: float = None,
    estim_r: float = None,
    estim_m: float = None,
) -> None:
    print("|" + "-" * 37 + "|")
    print("| Parameter | Real value | Estimation |")
    print("|" + "-" * 11 + "|" + "-" * 12 + "|" + "-" * 12 + "|")
    print(
        "|"
        + f"{'R_l (Ohm)':^11}"
        + "|"
        + f"{real_r:^12.3f}"
        + "|"
        + f"{estim_r:^12.3f}"
        + "|"
    )
    print("|" + "-" * 11 + "|" + "-" * 12 + "|" + "-" * 12 + "|")
    print(
        "|"
        + f"{'M (µH)':^11}"
        + "|"
        + f"{real_m:^12.2f}"
        + "|"
        + f"{estim_m:^12.2f}"
        + "|"
    )
    print("|" + "-" * 37 + "|" + "\n")
