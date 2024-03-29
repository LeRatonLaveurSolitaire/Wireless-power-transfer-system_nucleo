"""
Plot functions to diplay recieved and processed data
"""
import matplotlib.pyplot as plt
import numpy as np

def plot_signals(noisy_current:list, noisy_voltage:list,sampling_period:float) -> None:
    """Function that plot the signals once recieved through the serial port

    Args:
        noisy_current (list): noisy current signal
        noisy_voltage (list): noisy current signal
    """

    time = [i*sampling_period for i in range(len(noisy_current))]

    # Plotting currents
    plt.figure(figsize=(15, 8))

    plt.subplot(2, 1, 1)
    plt.plot(time,noisy_current, label='Noisy Current')
    plt.title('Noisy Current Signals')
    plt.xlabel('Time')
    plt.ylabel('Current')
    plt.legend()

    # Plotting voltage
    plt.subplot(2, 1, 2)
    plt.plot(time,noisy_voltage, label='Noisy Voltage')
    plt.title('Noisy Voltage Signal')
    plt.xlabel('Time')
    plt.ylabel('Voltage')
    plt.legend()

    plt.tight_layout()
    plt.show()

def plot_bode(impedances:list, frequencies:np.array, names:list = None, forms:list = None):
    
    min_freq, max_freq = 25_000,250_000
    
    plt.figure(figsize=(9, 6))
    
    # Magnitude plot
    plt.subplot(2, 1, 1)
    for i,impedance in enumerate(impedances):
        magnitudes = [abs(c) for c in impedance]
        name,form = None,''
        if names:
            name = names[i]
        if forms:
            form = forms[i]
        plt.semilogx(frequencies, 20 * np.log10(magnitudes),form,label=name)
    plt.title('Bode Plot')
    plt.ylabel('Magnitude (dB)')
    plt.legend()
    plt.grid()
    plt.xlim(min_freq, max_freq)

    # Phase plot
    plt.subplot(2, 1, 2)
    for i,impedance in enumerate(impedances):
        phases = [np.angle(c) * 180 / np.pi for c in impedance]
        name,form = None,''
        if names:
            name = names[i]
        if forms:
            form = forms[i]
        plt.semilogx(frequencies, phases,form,label=name)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Phase (degrees)')
    plt.legend()
    plt.grid()
    plt.xlim(min_freq, max_freq)

    plt.show()