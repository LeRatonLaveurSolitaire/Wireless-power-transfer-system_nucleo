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
C1 = 146e-9     # 1 / ((2 * np.pi * 85_000) ** 2 * L1)
R1 = 0.075

# Neural network parameters
model_path = "script/most_accurate_model.pt"

def main() -> None:
    """
    Main function.
    """

    ser = serial.Serial(PORT, baudrate=115200, timeout=0.1) # Open Serial port

    print("Ready to receive data")

    # Wait for data

    received = ser.readline().decode()
    while received == "":
        received = ser.readline().decode()
    print(received,end='')
    noisy_current, noisy_voltage = [],[]

    # Receive the data

    while received != "":
        received = ser.readline().decode()
        if ',' in received:
            print(received,end='')
            i,nc,nv = received.split(',')
            noisy_current.append(int(nc)*Q)
            noisy_voltage.append(int(nv))

    fft_c,fft_v = [],[]

    while received == "":
        received = ser.readline().decode()
    
    while received != "":
        received = ser.readline().decode()
        if ',' in received:
            print(received,end='')
            i,c_r,c_i,v_r,v_i = received.split(',')
            fft_c.append(float(c_r)+1j*float(c_i))
            fft_v.append(float(v_r)+1j*float(v_i))

    recieved_impedance = []

    while received == "":
        received = ser.readline().decode()
    
    while received != "":
        received = ser.readline().decode()
        if ',' in received:
            print(received,end='')
            i,impedance_r,impedance_i= received.split(',')
            recieved_impedance.append(float(impedance_r)+1j*float(impedance_i))


    ser.close()

    # Plot the data

    # plot_signals(noisy_current=noisy_current, noisy_voltage=noisy_voltage,sampling_period=sampling_period)

    # Process the data

    sys_impedance,sys_frequencies = extract_impedance(
        noisy_current=noisy_current,
        noisy_voltage=noisy_voltage,
        sampling_period=sampling_period,
        tau=6e-6,
    )

    smooth_sys_impedance = fractional_decade_smoothing_impedance(
        impedances=sys_impedance,
        frequencies=sys_frequencies,
        fractional_factor=1.04,
    )

    # adjust the phase so the mean phase between 75000 and 95000 is 0 wich should be the case in the real system
    phase_gain = -np.mean(np.angle(smooth_sys_impedance[np.where((sys_frequencies  < 95_000) & (sys_frequencies > 75_000))]))
    
    smooth_sys_impedance = [
        impedance * np.exp(1j * phase_gain) for impedance in smooth_sys_impedance
    ]


    # system_impedance_MCU = np.array(fft_v)/np.array(fft_c)
    # tau = 6e-6
    # system_impedance_MCU = np.array(
    #     [
    #         system_impedance_MCU[i+1]*np.exp(- 1j * np.pi * 2 * sys_frequencies[i] * tau + 1j*np.pi)
    #         for i in range(len(system_impedance_MCU)-1)
    #     ]
    # )

    # smooth_sys_impedance_MCU = fractional_decade_smoothing_impedance(
    #     impedances=system_impedance_MCU,
    #     frequencies=sys_frequencies,
    #     fractional_factor=1.04,
    # )

    

    # phase_gain_MCU = -np.mean(np.angle(smooth_sys_impedance_MCU[np.where((sys_frequencies  < 95_000) & (sys_frequencies > 75_000))]))
    # smooth_sys_impedance_MCU = [
    #     impedance * np.exp(1j * phase_gain_MCU) for impedance in smooth_sys_impedance_MCU
    # ]

    smooth_sys_impedance_MCU = np.array(recieved_impedance)

    f0 = 85000
    L1 = 24 * 1e-6
    C1 = 146e-9 #1 / ((2 * np.pi * f0) ** 2 * L1)
    R1 = 0.075 + 0.054
    # M = 16.27 * 1e-6 # for a 5mm gap
    M = 5.2345e-06 # for a 20mm gap
    # M = 6.04e-6
    L2 = 24 * 1e-6
    C2 = 146e-9 #1 / ((2 * np.pi * f0) ** 2 * L2)
    R2 = 0.075
    R_l = 1.7


    primary_s = wpt.transmitter(L=L1, C_s=C1, R=R1)
    secondary_s = wpt.reciever(L=L2, C_s=C2, R=R2, R_l=R_l)
    wpt_system = wpt.total_system(
        transmitter=primary_s, reciever=secondary_s, M=M, name="model"
    )
    model_impedance = np.array([wpt_system.impedance(freq) for freq in sys_frequencies])
    print(smooth_sys_impedance[0],smooth_sys_impedance_MCU[0],fft_c[1],fft_v[1])
    plot_bode([model_impedance,smooth_sys_impedance,smooth_sys_impedance_MCU],sys_frequencies,forms=['','x','x'],names=['Model','Estimation_PC','Estimation_MCU'])

    # Create the NN input tensor

    input_tensor = nn_input_tensor(
        sys_impedance=smooth_sys_impedance_MCU,
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