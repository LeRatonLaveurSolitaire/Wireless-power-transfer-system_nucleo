"""
Script that open serial port, recieve data from the MCU and plot them.
"""

import serial
import matplotlib.pyplot as plt
import numpy as np
import torch
from torch import tensor
import sys
import os

path_to_model = os.path.join(sys.path[0], "..", "neural_network")
sys.path.insert(0, path_to_model)

from model_class import NN_model
from identification_functions import *
from plot_function import *
import wpt_system_class as wpt

# Board parameters
PORT = "COM3"
sampling_period = 1e-6

# WPT system parameters
L1 = 24e-6
C1 = 146e-9  # 1 / ((2 * np.pi * 85_000) ** 2 * L1)
R1 = 0.075

# Neural network parameters
model_path = "neural_network/model.pt"


def main() -> None:
    """
    Main function.
    """

    ser = serial.Serial(PORT, baudrate=115200, timeout=0.1)  # Open Serial port

    print("Ready to receive data")

    # Wait for data

    received = ser.readline().decode()

    while received == "":
        received = ser.readline().decode()

    print("Receiving data, please wait...")

    noisy_current, noisy_voltage = [], []

    while received != "":
        received = ser.readline().decode()
        if "," in received:
            print(received, end="")
            i, nc, nv = received.split(",")
            noisy_current.append(float(nc))
            noisy_voltage.append(float(nv))

    while received == "":
        received = ser.readline().decode()

    recieved_impedance = []

    while received != "":
        received = ser.readline().decode()
        if "," in received:
            i, impedance_r, impedance_i = received.split(",")
            recieved_impedance.append(float(impedance_r) + 1j * float(impedance_i))

    recieved_tensor = []

    while received == "":
        received = ser.readline().decode()

    while received != "":
        received = ser.readline().decode()
        if "," in received:
            i, val = received.split(",")
            recieved_tensor.append(float(val))

    ser.close()

    plot_signals(
        noisy_current=(np.array(noisy_current)),
        noisy_voltage=(np.array(noisy_voltage)),
        sampling_period=sampling_period,
    )

    system_impedance = np.array(recieved_impedance)

    sys_frequencies = np.fft.rfftfreq(2 * (len(system_impedance)), sampling_period)[:-1]
    sys_frequencies[0] = 1

    smooth_sys_impedance = fractional_decade_smoothing_impedance(
        impedances=system_impedance,
        frequencies=sys_frequencies,
        fractional_factor=1.06,
    )

    plot_bode(
        [
            system_impedance,
            smooth_sys_impedance,
        ],
        sys_frequencies,
        forms=["x", "x"],
        names=["raw impedance", "smoothed impedance"],
    )

    f0 = 85000
    L1 = 24 * 1e-6
    C1 = 152e-9  # 1 / ((2 * np.pi * f0) ** 2 * L1)
    R1 = 0.075
    # M = 16.27 * 1e-6 # for a 5mm gap
    # M = 5.2345e-06  # for a 20mm gap
    M = 7.4129e-06  # for a 15mm gap
    # M = 6.01e-6
    L2 = 24 * 1e-6
    C2 = 151e-9  # 1 / ((2 * np.pi * f0) ** 2 * L2)
    R2 = 0.4
    R_l = 3

    primary_s = wpt.transmitter(L=L1, C_s=C1, R=R1)
    primary_s = wpt.transmitter(L=L1, C_s=C1, R=R1)
    secondary_s = wpt.reciever(L=L2, C_s=C2, R=R2, R_l=R_l)
    wpt_system = wpt.total_system(
        transmitter=primary_s, reciever=secondary_s, M=M, name="model"
    )
    model_impedance = np.array([wpt_system.impedance(freq) for freq in sys_frequencies])

    # Create the NN input tensor

    input_tensor = nn_input_tensor(
        sys_impedance=smooth_sys_impedance,
        sys_frequencies=sys_frequencies,
        L1=L1,
        R1=R1,
        C1=C1,
    )
    # for i in range(30):
    #     print(f"{float(input_tensor[i]):.3f}, {recieved_tensor[i]:.3f}")

    # Import the neural network

    neural_network = NN_model(input_size=30, output_size=2)
    checkpoint = torch.load(model_path)
    neural_network.load_state_dict(checkpoint)

    # Compute NN inferance

    with torch.inference_mode():
        output_tensor = neural_network(tensor(input_tensor))
        # output_tensor = neural_network(tensor(recieved_tensor))

    R = delinearise_R_l(output_tensor[0].item())
    M = delinearise_M(output_tensor[1].item())

    # Print the parameters estimation

    print(f"R = {R:5.2f} Ohm")
    print(f"M = {M*1e6:5.2f} µH")
    # print(f"{phase_gain = }")

    secondary_s = wpt.reciever(L=L2, C_s=C2, R=R2, R_l=R)
    wpt_system = wpt.total_system(
        transmitter=primary_s, reciever=secondary_s, M=M, name="model"
    )
    model2_impedance = np.array(
        [wpt_system.impedance(freq) for freq in sys_frequencies]
    )

    err = 0
    for i in range(199, 578):
        err += (
            abs(
                (
                    abs(20 * np.log(model_impedance[i]))
                    - abs(20 * np.log(smooth_sys_impedance[i]))
                )
                / abs(20 * np.log(model_impedance[i]))
            )
            + abs(
                (
                    np.angle(model_impedance[i])
                    - np.angle(smooth_sys_impedance_centered[i])
                )
                / np.angle(model_impedance[i])
            )
        ) / 2
    err /= 578 - 199
    print(f"relative error on the impedance plot : {err:.2%}")

    plot_bode(
        [
            model_impedance,
            # smooth_sys_impedance,
            smooth_sys_impedance_centered,
            # model2_impedance,
        ],
        sys_frequencies,
        forms=[
            "",
            "x",
            # "r",
        ],
        names=[
            "Model",
            "mesurment",
            # "Model w/ estimated param",
        ],
        f0=85_000,
        tensor=recieved_tensor,
    )


if __name__ == "__main__":
    while True:
        main()
