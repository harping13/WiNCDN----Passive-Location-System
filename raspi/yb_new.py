# Import necessary modules and initialize constants
import serial
import asyncio
from threading import Thread
import re

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
READ_TIMEOUT = 1

# Initialize global variables
last_qos_time = None
delays = []
temp_delays = []
temp_rssi_sum = 0

# Function to clean ANSI escape sequences from input string
def clean_ansi_escape_sequences(input_str):
    ansi_escape_pattern = re.compile(r'\x1b\[[0-9;]*m')
    return ansi_escape_pattern.sub('', input_str)

# Async function to save averaged delays and RSSI values to a file
async def save_to_file_async(data, filename, stop_event, loop):
    with open(filename, 'w') as outfile:
        outfile.write("Delay (usec),RSSI\n")
        for delay, rssi in data:
            outfile.write(f"{delay},{rssi}\n")
    print("Data saved and exiting...")
    stop_event.set()  # Notify to stop
    loop.call_soon_threadsafe(loop.stop)  # Schedule the loop to stop

# Async function to process delay and RSSI, averaging and storing them
async def process_data_async(delay, rssi, loop, stop_event):
    global last_qos_time, delays, temp_delays, temp_rssi_sum
    temp_delays.append(delay)
    temp_rssi_sum += rssi
    if len(temp_delays) == 10:
        avg_delay = sum(temp_delays) / len(temp_delays)
        avg_rssi = temp_rssi_sum / len(temp_delays)
        delays.append((avg_delay, avg_rssi))
        temp_delays.clear()
        temp_rssi_sum = 0
        #print(len(delays))
        if len(delays) == 30:
            await save_to_file_async(delays.copy(), "delays_rssi_filtered.csv", stop_event, loop)
            delays.clear()

# Function to read serial data, process QoS and ACK packets, and extract timestamp and RSSI
def read_serial_data(serial_port, baud_rate, timeout, loop, stop_event):
    global last_qos_time
    with serial.Serial(serial_port, baud_rate, timeout=timeout) as ser:
        while not stop_event.is_set():
            if ser.in_waiting:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                data = clean_ansi_escape_sequences(data)
                if "QoS" in data:
                    try:
                        last_qos_time = int(data.split('Timestamp: ')[-1].split(',')[0])
                    except ValueError as e:
                        print(f"Error processing QoS line: {data} - {e}")
                elif "ACK" in data and last_qos_time is not None:
                    try:
                        ack_time, rssi = [int(x) for x in re.search(r"Timestamp: (\d+), RSSI: (-?\d+)", data).groups()]
                        delay = ack_time - last_qos_time - 76200
                       # print(delay)  # Print delay for diagnostics
                        if abs(delay) <= 100:
                            asyncio.run_coroutine_threadsafe(process_data_async(delay, rssi, loop, stop_event), loop)
                    except ValueError as e:
                        print(f"Error processing ACK line: {data} - {e}")

# Main function to initialize and start the event loop and serial reading thread
def main():
    stop_event = asyncio.Event()
    loop = asyncio.get_event_loop()
    read_thread = Thread(target=read_serial_data, args=(SERIAL_PORT, BAUD_RATE, READ_TIMEOUT, loop, stop_event))
    read_thread.start()

    try:
        loop.run_forever()
    finally:
        stop_event.set()
        read_thread.join()
        loop.close()

# Commenting out the function call to avoid automatic execution
if __name__ == "__main__":
     main()

