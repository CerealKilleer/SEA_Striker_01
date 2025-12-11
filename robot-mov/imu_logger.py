"""
IMU Data Logger
Reads IMU data from ESP32 serial port and saves to CSV file
"""

import serial
import csv
import re
from datetime import datetime
import argparse
import sys

def parse_imu_line(line):
    """
    Parse ESP-IDF log line with IMU data.
    Expected format: I (timestamp) task_name: ax, ay, az, gx, gy, gz
    Example: I (12345) enc_gk: 0.15, 0.23, -9.81, 0.12, -0.05, 0.03
    """
    # Pattern for ESP-IDF log: I (number) task_name: float, float, float, float, float, float
    pattern = r'I \(\d+\) \w+: ([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+),?\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+)'
    
    match = re.search(pattern, line)
    if match:
        return [float(match.group(i)) for i in range(1, 7)]
    return None

def main():
    parser = argparse.ArgumentParser(description='Log IMU data to CSV file')
    parser.add_argument('--port', type=str, default='COM3', 
                       help='Serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)')
    parser.add_argument('--baudrate', type=int, default=115200,
                       help='Baudrate (default: 115200)')
    parser.add_argument('--output', type=str, 
                       default=f'imu_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv',
                       help='Output CSV filename')
    
    args = parser.parse_args()
    
    # Setup CSV file
    csv_file = open(args.output, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    
    # Write header
    csv_writer.writerow(['timestamp_s', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])
    
    print(f"Opening serial port {args.port} at {args.baudrate} baud...")
    print(f"Saving data to {args.output}")
    print("Press Ctrl+C to stop logging\n")
    
    try:
        # Open serial port
        ser = serial.Serial(args.port, args.baudrate, timeout=1)
        
        # Clear any existing data in buffer
        ser.reset_input_buffer()
        
        samples_count = 0
        
        while True:
            try:
                # Read line from serial port
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line:
                    # Try to parse IMU data
                    imu_data = parse_imu_line(line)
                    
                    if imu_data:
                        # Get timestamp in seconds.milliseconds
                        now = datetime.now()
                        timestamp = now.timestamp()  # Unix timestamp with milliseconds
                        
                        # Write to CSV
                        csv_writer.writerow([timestamp] + imu_data)
                        csv_file.flush()  # Ensure data is written immediately
                        
                        samples_count += 1
                        
                        # Print every 10 samples to avoid cluttering terminal
                        if samples_count % 10 == 0:
                            print(f"[{samples_count}] ax={imu_data[0]:7.2f} ay={imu_data[1]:7.2f} az={imu_data[2]:7.2f} "
                                  f"gx={imu_data[3]:7.2f} gy={imu_data[4]:7.2f} gz={imu_data[5]:7.2f}")
                    else:
                        # Print other serial output (for debugging)
                        if line and not line.startswith('I ('):
                            print(f"Debug: {line}")
                            
            except UnicodeDecodeError:
                continue
                
    except serial.SerialException as e:
        print(f"\nError opening serial port: {e}")
        print("Available ports might be: COM1, COM3, COM4, etc.")
        sys.exit(1)
        
    except KeyboardInterrupt:
        print(f"\n\nLogging stopped. Total samples: {samples_count}")
        print(f"Data saved to {args.output}")
        
    finally:
        csv_file.close()
        if 'ser' in locals():
            ser.close()

if __name__ == '__main__':
    main()
