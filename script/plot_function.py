"""
Plot functions to diplay recieved and processed data
"""

import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
import numpy as np

font = {"size": 12}

plt.rc("font", **font)


def plot_signals(
    noisy_current: list, noisy_voltage: list, sampling_period: float
) -> None:
    """Function that plot the signals once recieved through the serial port

    Args:
        noisy_current (list): noisy current signal
        noisy_voltage (list): noisy current signal
    """

    time = [i * sampling_period for i in range(len(noisy_current))]

    # Plotting currents
    plt.figure(figsize=(15, 8))

    plt.subplot(2, 1, 1)
    plt.plot(time, noisy_current, label="Noisy Current")
    plt.title("Noisy Current Signals")
    plt.xlabel("Time")
    plt.ylabel("Current")
    plt.legend()

    # Plotting voltage
    plt.subplot(2, 1, 2)
    plt.plot(time, noisy_voltage, label="Noisy Voltage")
    plt.title("Noisy Voltage Signal")
    plt.xlabel("Time")
    plt.ylabel("Voltage")
    plt.legend()

    plt.tight_layout()
    plt.show()


def plot_bode(
    impedances: list,
    frequencies: np.array,
    names: list = None,
    forms: list = None,
    f0: float = None,
    tensor=None,
):

    min_freq, max_freq = 25_000 / 1000, 250_000 / 1000
    indexs = [199, 215, 232, 250, 270, 291, 314, 339, 366, 395, 426, 459, 496, 535, 577]
    plt.figure(figsize=(9, 6))

    # Magnitude plot
    ax = plt.subplot(2, 1, 1)
    for i, impedance in enumerate(impedances):
        magnitudes = [abs(c) for c in impedance]
        name, form = None, ""
        if names:
            name = names[i]
        if forms:
            form = forms[i]
        plt.semilogx(frequencies / 1000, 20 * np.log10(magnitudes), form, label=name)
        if name == "mesurment":
            plt.semilogx(
                [frequencies[i] / 1000 for i in indexs],
                [20 * np.log10(magnitudes[i]) for i in indexs],
                "og",
                label="input tensor",
            )
    if f0 != None:
        ax.axvline(x=f0 / 1000, color="black", linestyle=":")
        y_bot1 = 0
        y_top1 = 50
        ax.text(
            f0 / 1000,
            y_bot1 - (y_top1 - y_bot1) * 0.1,
            r"$f0$",
            color="black",
            ha="center",
            va="center",
        )

    if tensor != None:
        freqs = np.geomspace(50000, 144500, num=15, dtype=np.int64)
        plt.semilogx(freqs, 20 * np.log10(tensor[::2]), ".", label="tensor")

    plt.title("Bode Plot")
    plt.ylabel("Magnitude (dB)")
    plt.legend()
    plt.grid()
    plt.xlim(min_freq, max_freq)
    plt.ylim(0, 50)
    ax.xaxis.set_major_formatter(ScalarFormatter(useMathText=False))
    # Phase plot
    ax = plt.subplot(2, 1, 2)
    for i, impedance in enumerate(impedances):
        phases = [np.angle(c) * 180 / np.pi for c in impedance]
        name, form = None, ""
        if names:
            name = names[i]
        if forms:
            form = forms[i]
        plt.semilogx(frequencies / 1000, phases, form, label=name)
        if name == "mesurment":
            plt.semilogx(
                [frequencies[i] / 1000 for i in indexs],
                [phases[i] for i in indexs],
                "og",
                label="input tensor",
            )
    if f0 != None:
        ax.axvline(x=f0 / 1000, color="black", linestyle=":")
        y_bot1 = ax.get_ylim()[0]
        y_top1 = ax.get_ylim()[1]
        ax.text(
            f0 / 1000,
            y_bot1 - (y_top1 - y_bot1) * 0.1,
            r"$f0$",
            color="black",
            ha="center",
            va="center",
        )

    if tensor != None:
        freqs = np.geomspace(50000, 144500, num=15, dtype=np.int64)
        plt.semilogx(freqs, 180 / np.pi * np.array(tensor[1::2]), ".", label="tensor")

    plt.xlabel("Frequency (kHz)")
    plt.ylabel("Phase (degrees)")
    plt.legend()
    plt.grid()
    plt.xlim(min_freq, max_freq)
    ax.xaxis.set_major_formatter(ScalarFormatter(useMathText=False))
    plt.show()
