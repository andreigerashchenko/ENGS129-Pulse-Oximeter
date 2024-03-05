import serial
import matplotlib
import matplotlib.pyplot as plt
from collections import deque
import time

ADC_RANGE = 2**12 - 1  # 12-bit ADC
PLOT_MIN_Y = 0.3 # Minimum value for the y-axis
PLOT_MAX_Y = 3.6 # Maximum value for the y-axis
PLOT_TIME = 5  # Plot the last 5 seconds of data
DATA_FREQUENCY = 60  # Data frequency in Hz

BAUD = 2000000
ch1_data = deque(maxlen=(PLOT_TIME * DATA_FREQUENCY))  # Keep the last 5 seconds of channel 1 data
ch2_data = deque(maxlen=(PLOT_TIME * DATA_FREQUENCY))  # Keep the last 5 seconds of channel 2 data
ch1_timestamps = deque(maxlen=(PLOT_TIME * DATA_FREQUENCY))  # Keep the last 5 seconds of channel 1 timestamps
ch2_timestamps = deque(maxlen=(PLOT_TIME * DATA_FREQUENCY))  # Keep the last 5 seconds of channel 2 timestamps
peaks_data = deque(maxlen=(PLOT_TIME * DATA_FREQUENCY))  # Keep the last 5 seconds of peaks data
peaks_timestamps = deque(maxlen=(PLOT_TIME * DATA_FREQUENCY))  # Keep the last 5 seconds of peaks timestamps

red_duty_cycle = 0
ir_duty_cycle = 0
peak = 0
heartrate = 0
spo2 = 0
heart_period = 0

should_update = False # Flag to indicate that the plot should be updated
samples = 0  # Number of samples received in last transmission

# Find the serial port with the microcontroller connected
port = None
       

if __name__ == "__main__":
    for i in range(256):
        try:
            port = serial.Serial(f'COM{i}', baudrate=BAUD, timeout=0.1)
            break
        except serial.SerialException:
            pass

    if port is None:
        print("No microcontroller found.")
    else:
        print(f"Connected to {port.name} at {port.baudrate} baud.")

        # Start receiving and plotting lines of data
        fig, ax = plt.subplots()
        line1, = ax.plot(ch1_timestamps, ch1_data, label='Red Light', color='red')
        line2, = ax.plot(ch2_timestamps, ch2_data, label='Infrared Light', color='blue')
        line3, = ax.plot(peaks_timestamps, peaks_data, label='Peaks', color='green')
        ax.legend()

        plt.ion()
        plt.show()

        receiving = False

        while True:
            if should_update:
                if len(ch1_timestamps) > 0:
                    line1.set_xdata(ch1_timestamps)
                    line1.set_ydata(ch1_data)
                if len(ch2_timestamps) > 0:
                    line2.set_xdata(ch2_timestamps)
                    line2.set_ydata(ch2_data)
                if len(peaks_timestamps) > 0:
                    line3.set_xdata(peaks_timestamps)
                    line3.set_ydata(peaks_data)
                if len(ch1_timestamps) > 0 or len(ch2_timestamps) > 0 or len(peaks_timestamps) > 0:
                    ax.relim()
                    # ax.autoscale_view()
                    if len(ch1_timestamps) > 0:
                        ax.set_xlim(ch1_timestamps[0], ch1_timestamps[-1])
                    elif len(ch2_timestamps) > 0:
                        ax.set_xlim(ch2_timestamps[0], ch2_timestamps[-1])
                    elif len(peaks_timestamps) > 0:
                        ax.set_xlim(peaks_timestamps[0], peaks_timestamps[-1])
                    ax.set_ylim(PLOT_MIN_Y, PLOT_MAX_Y)
                    ax.set_xlabel('Time (ms)')
                    ax.set_ylabel('Voltage (V)')
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    plt.pause(0.01)
                should_update = False

            line = port.readline()
            if len(line) == 0:
                continue
            if not receiving and line[0] == b'P':
                print(line)
            # Check if line is DATA_START\n
            if not receiving and line == b'DATA_START\n':
                print("Data start")
                receiving = True
                samples = 0
            elif receiving and line == b'DATA_END\n':
                print("Data end")
                receiving = False
                
                line = ''
                
                print(f"Samples received: {samples}")
                print(f"Red Duty Cycle: {red_duty_cycle}")
                print(f"IR Duty Cycle: {ir_duty_cycle}")
                # print(f"Peak: {peak}")
                print(f"Heart Rate: {heartrate}")
                print(f"SPO2: {spo2}")
                print(f"Heart Period: {heart_period}")
                # Update the plot
                should_update = True
            elif receiving and len(line) == 8:
                timestamp = int.from_bytes(line[:4], byteorder='little')
                data_MSB = line[4]
                data_LSB = line[5]
                channel = line[6]
                full_data = (data_MSB << 8) | data_LSB

                if channel == 1:
                    ch1_data.append(full_data / ADC_RANGE * PLOT_MAX_Y)
                    ch1_timestamps.append(timestamp)
                    samples += 1
                elif channel == 2:
                    ch2_data.append(full_data / ADC_RANGE * PLOT_MAX_Y)
                    ch2_timestamps.append(timestamp)
                    samples += 1
                elif channel == 3:
                    red_duty_cycle = data_MSB
                    ir_duty_cycle = data_LSB
                elif channel == 4:
                    peak = full_data
                elif channel == 5:
                    # heartrate is 8 bit value in MSB (bpm)
                    heartrate = data_MSB
                    # spo2 is 8 bit value in LSB (%)
                    spo2 = data_LSB
                elif channel == 7:
                    peaks_data.append(full_data / ADC_RANGE * PLOT_MAX_Y)
                    peaks_timestamps.append(timestamp)
                elif channel == 8:
                    heart_period = full_data
                
                # print(f"T: {timestamp} | MSB: {data_MSB} | LSB: {data_LSB} | Data: {full_data} | Ch: {channel}")