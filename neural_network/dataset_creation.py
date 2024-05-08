"""A script that create a dataset.

Data have the following shape :
input : [|Z(w1)|, arg(Z(w1)),...,|Z(wn)|,arg(Z(wn))] with (w1, ... ,wn) the list of frequencies at which the impedance is mesured
output : [R_l,M]
"""

import random
import numpy as np
import sys
import os

path_to_utils = os.path.join(sys.path[0], "..", "script")
sys.path.insert(0, path_to_utils)

import wpt_system_class as wpt
from dataset_class import CustomDataset


def main() -> None:
    """Main script function"""

    # data parameters

    f0 = 85_000
    L1 = 24e-6
    R1 = 0.075
    C1 = 146e-9  # 1 / ((2 * np.pi * f0) ** 2 * L1)
    R2 = 0.075
    L2 = 24e-6
    C2 = 146e-9  # 1 / ((2 * np.pi * f0) ** 2 * L2)
    M_bound = [0.05 * (L1 * L2) ** 0.5, (L1 * L2) ** 0.5]
    R_l_bound = [0.1, 10]
    C_2_bound = [
        C2 * 79 / 85,
        C2 * 90 / 85,
    ]  # enable to vary the resonance frequency between 79kHz and 90kHz
    sigma = 0.05

    frequencies_to_test = np.geomspace(50000, 144500, num=15, dtype=np.int64)

    # dataset parameters

    data = []
    dataset_size = 100_000

    for i in range(dataset_size):
        M = random.uniform(*np.log10(M_bound))
        M = 10**M
        C_2 = random.uniform(*C_2_bound)
        R_l = random.uniform(*np.log10(R_l_bound))
        R_l = 10**R_l

        f2 = 1 / (2 * np.pi * (L2 * C_2) ** 0.5)

        # Normalizing dataset do have value between [-1;1]
        R_l_data = (np.log10(R_l)) * 10
        M_data = (np.log10(M / (0.1 * (L1 * L2) ** 0.5))) * 10
        f2_data = (f2 - 85000) / 500

        output_list = [R_l_data, M_data]

        primary = wpt.transmitter(topology="S", L=L1, C_s=C1, R=R1)
        secondary = wpt.reciever(topology="S", L=L2, R=R2, C_s=C_2, R_l=R_l)
        wpt_system = wpt.total_system(transmitter=primary, reciever=secondary, M=M)

        input_list = []

        for j, frequency in enumerate(frequencies_to_test):
            impedence = wpt_system.impedance(frequency=frequency)
            # Z1 = R1 + 1j * L1 * 2 * np.pi * frequency - 1j / (C1 * 2 * np.pi * frequency)
            Z2 = impedence  # - Z1
            input_list.append(np.absolute(Z2) * (random.gauss(1, sigma)))
            input_list.append(np.angle(Z2) * random.gauss(1, sigma))

        data.append((input_list, output_list))

        if i % 1000 == 0:
            print(f"Creation of the dataset... {i/dataset_size:03.0%}")

    print("Creation of the dataset... 100%")
    print("Saving the dataset as a pickle file")
    dataset = CustomDataset(data=data)
    dataset.save(file_path=os.path.join("neural_network", "dataset.pkl"))
    print("Done !")


if __name__ == "__main__":
    main()
