import serial
import struct
import csv
from datetime import datetime

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
TOTAL_CHANNELS_SENT = 16
# List the channel numbers you want to save (0-15)
CHANNELS_TO_LOG = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11] 

# --- ADC & Voltage Conversion ---
# [MODIFIED] This calculation now matches the Teensy code's Gain setting of 0 (+/- 6.144V)
VOLTS_PER_BIT = 6.144 / 32767.0

# --- CSV File Setup ---
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"data_final_corrected_{timestamp}.csv"
header = ['timestamp_us'] + [f'CH{i}_V' for i in CHANNELS_TO_LOG]

print(f"Connecting to {SERIAL_PORT}...")
print(f"Logging selected channels: {CHANNELS_TO_LOG}")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)
        print(f"Successfully opened {filename}. Logging selected channels...")
        print("Press Ctrl+C to stop.")

        while True:
            if ser.read(1) == b'\xaa':
                if ser.read(1) == b'\x55':
                    data_bytes = ser.read(4 + TOTAL_CHANNELS_SENT * 2) 
                    
                    if len(data_bytes) == (4 + TOTAL_CHANNELS_SENT * 2):
                        unpacked_data = struct.unpack(f'<L{TOTAL_CHANNELS_SENT}h', data_bytes)
                        
                        timestamp_us = unpacked_data[0]
                        adc_raw_values = unpacked_data[1:]
                        all_voltages = [val * VOLTS_PER_BIT for val in adc_raw_values]
                        
                        voltages_to_log = [all_voltages[i] for i in CHANNELS_TO_LOG]
                        
                        writer.writerow([timestamp_us] + voltages_to_log)

except serial.SerialException as e:
    print(f"Error: Could not open serial port {SERIAL_PORT}. {e}")
    print("Is the device connected? Do you have the correct permissions (e.g., 'sudo adduser $USER dialout')?")
except KeyboardInterrupt:
    print("\nLogging stopped by user. CSV file saved.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")
