"""
Script that open serial port, recieve data from the MCU and plot them.
"""
import serial
import matplotlib.pyplot as plt

PORT = "COM3"
Te =  1/1_000_000

def plot_signals(clean_current:list, noisy_current:list, noisy_voltage:list) -> None:
    """Function that plot the signals once recieved through the serial port

    Args:
        clean_current (list): clean current signal
        noisy_current (list): noisy current signal
        noisy_voltage (list): noisy current signal
    """
    # Plotting currents
    plt.figure(figsize=(10, 8))
    time = [i*Te for i in range(len(clean_current))]
    plt.subplot(3, 1, 1)
    plt.plot(time,clean_current, label='Clean Current')
    plt.title('Current Signals')
    plt.xlabel('Time')
    plt.ylabel('Current')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time,noisy_current, label='Noisy Current')
    plt.title('Current Signals')
    plt.xlabel('Time')
    plt.ylabel('Current')
    plt.legend()

    # Plotting voltage
    plt.subplot(3, 1, 3)
    plt.plot(time,noisy_voltage, label='Noisy Voltage')
    plt.title('Voltage Signal')
    plt.xlabel('Time')
    plt.ylabel('Voltage')
    plt.legend()

    plt.tight_layout()
    plt.show()


def main()-> None:
    """
    Main function.
    """
    ser = serial.Serial(PORT, baudrate=115200, timeout=5)
    recived = ser.readline().decode()
    print(recived)
    clean_current, noisy_current, noisy_voltage = [],[],[]
    while recived != "":
        recived = ser.readline().decode()
        if ',' in recived:
            print(recived)
            i,cc,nc,nv = recived.split(',')
            clean_current.append(int(cc))
            noisy_current.append(int(nc))
            noisy_voltage.append(int(nv))
    ser.close()
    plot_signals(clean_current, noisy_current, noisy_voltage)


if __name__ == "__main__":
    main()