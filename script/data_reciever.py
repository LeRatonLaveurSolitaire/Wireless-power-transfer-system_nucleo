"""
Script that open serial port, recieve data from the MCU and plot them.
"""
import serial
import matplotlib.pyplot as plt
import numpy as np
import torch
from torch import tensor

from nn_model import NN_model
from identification_functions import *
from plot_function import *
import wpt_system_class as wpt

# Board parameters 
PORT = "COM3"
sampling_period =  1e-6
Q = 6.6 / 4095 / 500 / 0.001    # Quantization factor (int -> Amp)

# WPT system parameters
L1 = 24e-6
C1 = 140e-9     # 1 / ((2 * np.pi * 85_000) ** 2 * L1)
R1 = 0.075

# Neural network parameters
model_path = "script/most_accurate_model.pt"

def main() -> None:
    """
    Main function.
    """

    ser = serial.Serial(PORT, baudrate=115200, timeout=0.1) # Open Serial port

    print("Waiting to receive data")

    # Wait for data

    received = ser.readline().decode()
    while received == "":
        received = ser.readline().decode()
    print(received,end='')

    clean_current, noisy_current, noisy_voltage = [],[],[]

    # Receive the data

    while received != "":
        received = ser.readline().decode()
        if ',' in received:
            print(received,end='')
            i,cc,nc,nv = received.split(',')
            clean_current.append(int(cc)*Q)
            noisy_current.append(int(nc)*Q)
            noisy_voltage.append(-int(nv))
    ser.close()

    # Plot the data

    plot_signals(clean_current, noisy_current, noisy_voltage,sampling_period)

    # Process the data

    sys_impedance,sys_frequencies = extract_impedance(
        clean_current=clean_current,
        noisy_current=noisy_current,
        noisy_voltage=noisy_voltage,
        sampling_period=sampling_period
    )

    smooth_sys_impedance = fractional_decade_smoothing_impedance(
        impedances=sys_impedance,
        frequencies=sys_frequencies,
        fractional_factor=1.05,
    )

    f0 = 85000
    L1 = 24 * 1e-6
    C1 = 140e-9 #1 / ((2 * np.pi * f0) ** 2 * L1)
    R1 = 0.075
    # M = 16.27 * 1e-6 # for a 5mm gap
    M = 5.2345e-06 # for a 20mm gap
    L2 = 24 * 1e-6
    C2 = 142e-9 #1 / ((2 * np.pi * f0) ** 2 * L2)
    R2 = 0.075
    R_l = 1.25


    primary_s = wpt.transmitter(L=L1, C_s=C1, R=R1)
    secondary_s = wpt.reciever(L=L2, C_s=C2, R=R2, R_l=R_l)
    wpt_system = wpt.total_system(
        transmitter=primary_s, reciever=secondary_s, M=M, name="model"
    )
    model_impedance = np.array([wpt_system.impedance(freq) for freq in sys_frequencies])

    plot_bode([model_impedance,sys_impedance,smooth_sys_impedance],sys_frequencies,forms=['','.','x'],names=['model','raw','filtered'])

    # Create the NN input tensor

    input_tensor = nn_input_tensor(
        sys_impedance=smooth_sys_impedance,
        sys_frequencies=sys_frequencies,
        L1=L1,
        R1=R1,
        C1=C1,
    )

    # Import the neural network

    neural_network = NN_model(input_size=30, output_size=2)
    checkpoint = torch.load(model_path)
    neural_network.load_state_dict(checkpoint)

    # Compute NN inferance

    with torch.inference_mode():
        output_tensor = neural_network(input_tensor)

    R = delinearise_R_l(output_tensor[0].item())
    M = delinearise_M(output_tensor[1].item())

    # Print the parameters estimation

    print(f"M = {M*1e6:5.2f} µH")
    print(f"R = {R:5.2f} Ohm")

if __name__ == "__main__":
    main()