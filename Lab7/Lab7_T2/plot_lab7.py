#!/usr/bin/env python3
"""
Lab 7 Task 1/2 - Serial Data Plotter
"""

import sys
import os
import time

# Try to import serial with proper error handling
try:
    import serial
    print("✓ Serial module imported successfully")
    print(f"  Version: {serial.VERSION if hasattr(serial, 'VERSION') else 'unknown'}")
except ImportError as e:
    print(f"✗ Failed to import serial: {e}")
    print("  Please install pyserial: pip3 install pyserial")
    sys.exit(1)

import matplotlib.pyplot as plt
import numpy as np

# USING THE CORRECT PORT!
PORT = '/dev/ttyUSB0'  # Changed from ttyACM0 to ttyUSB0
BAUD = 115200
TIMEOUT = 2

print(f"\n=== Lab 7 Data Plotter ===")
print(f"Port: {PORT}")
print(f"Baud rate: {BAUD}")
print("-" * 30)

# Check if port exists
if not os.path.exists(PORT):
    print(f"⚠ Warning: Port {PORT} does not exist!")
    print("  Available ports:")
    os.system('ls -la /dev/tty* 2>/dev/null | grep -E "ttyUSB|ttyACM"')
    sys.exit(1)

# Try to connect
try:
    print(f"\nConnecting to {PORT}...")
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        timeout=TIMEOUT,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    
    # Clear any old data
    ser.reset_input_buffer()
    time.sleep(2)
    
    print(f"✓ Connected successfully!")
    print(f"  Waiting for data from STM32...\n")

except Exception as e:
    print(f"✗ Failed to connect: {e}")
    print("\nTroubleshooting:")
    print(f"1. Check if STM32 is connected: lsusb")
    print(f"2. Fix permissions: sudo chmod 666 {PORT}")
    print(f"3. Add user to dialout: sudo usermod -a -G dialout $USER")
    print("   Then log out and back in")
    sys.exit(1)

# Data storage
raw_data = []
filtered_data = []
sample_numbers = []
cnt = 0
in_data_block = False
data_count = 0

# Set up plot
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
fig.suptitle('Lab 7: ADC Data - Raw vs Filtered', fontsize=14)

print("Listening for data... Press Ctrl+C to stop")

# Main loop
try:
    while True:
        try:
            # Read a line
            line = ser.readline().decode('utf-8').strip()
            
            if not line:
                continue
                
            # Check for data markers
            if line == "=== DATA START ===":
                print(f"\n▶ New data batch started at {time.strftime('%H:%M:%S')}")
                raw_data = []
                filtered_data = []
                sample_numbers = []
                cnt = 0
                in_data_block = True
                data_count += 1
                continue
                
            elif line == "=== DATA END ===":
                print(f"  ◀ Batch complete: {cnt} samples received")
                in_data_block = False
                
                # Plot the data
                if cnt > 0:
                    # Clear axes
                    ax1.clear()
                    ax2.clear()
                    
                    # Plot raw data
                    ax1.plot(sample_numbers, raw_data, 'r-', linewidth=1.5, label='Raw ADC')
                    ax1.set_ylabel('Voltage (mV)')
                    ax1.set_title(f'Raw Signal (Batch {data_count})')
                    ax1.grid(True, alpha=0.3)
                    ax1.legend(loc='upper right')
                    ax1.set_ylim(0, 4000)
                    
                    # Plot filtered data
                    ax2.plot(sample_numbers, filtered_data, 'b-', linewidth=2, label='Filtered')
                    ax2.set_xlabel('Sample Number')
                    ax2.set_ylabel('Voltage (mV)')
                    ax2.set_title('Filtered Signal (Moving Average)')
                    ax2.grid(True, alpha=0.3)
                    ax2.legend(loc='upper right')
                    ax2.set_ylim(0, 4000)
                    
                    # Update plot
                    plt.tight_layout()
                    plt.draw()
                    plt.pause(0.1)
                    
                    # Print statistics
                    if len(raw_data) > 0:
                        print(f"  Raw range: {min(raw_data)}-{max(raw_data)} mV")
                        print(f"  Filtered range: {min(filtered_data)}-{max(filtered_data)} mV")
                continue
                
            # Parse data if we're in a block
            if in_data_block:
                try:
                    # Split the line
                    values = line.split(',')
                    if len(values) == 2:
                        raw = int(values[0])
                        filtered = int(values[1])
                        
                        raw_data.append(raw)
                        filtered_data.append(filtered)
                        sample_numbers.append(cnt)
                        cnt += 1
                        
                        # Print progress every 10 samples
                        if cnt % 10 == 0:
                            print(f"  Received {cnt} samples...", end='\r')
                            
                except ValueError as e:
                    print(f"  Invalid data: {line}")
                except Exception as e:
                    print(f"  Parse error: {e}")
                    
        except UnicodeDecodeError:
            # Skip invalid characters
            continue
        except serial.SerialException as e:
            print(f"\nSerial error: {e}")
            break
        except KeyboardInterrupt:
            print("\n\n✓ Stopped by user")
            break
        except Exception as e:
            print(f"\nUnexpected error: {e}")
            continue
            
except KeyboardInterrupt:
    print("\n\n✓ Stopped by user")
    
finally:
    # Clean up
    if 'ser' in locals():
        ser.close()
        print("✓ Serial port closed")
    
    # Keep plot window open
    plt.ioff()
    plt.show()
    
print("\nDone!")