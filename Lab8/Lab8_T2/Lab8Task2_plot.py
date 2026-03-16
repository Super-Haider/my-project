import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ============================================================
# CONFIGURATION — change COM port to match your system
# Windows example : 'COM3'
# Linux example   : '/dev/ttyUSB0' or '/dev/ttyACM0'
# macOS example   : '/dev/cu.usbmodem1234'
# ============================================================
SERIAL_PORT = '/dev/ttyUSB0'       # <-- CHANGE THIS to your actual port
BAUD_RATE   = 115200       # Must match huart2 baud rate in STM32 code
NUM_SAMPLES = 100          # How many samples to show on screen at once

# ============================================================
# Setup
# ============================================================
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

time_axis  = []   # x-axis sample counter
temp_data  = []   # y-axis temperature values
counter    = [0]  # mutable counter (used inside nested function)

fig, ax = plt.subplots()
line,   = ax.plot([], [], 'r.-', label='Temperature')

ax.set_title('I3G4250D Gyroscope — Temperature (Live)')
ax.set_xlabel('Sample Number')
ax.set_ylabel('Temperature (relative, °C)')
ax.set_ylim(-50, 50)         # Adjust if your readings fall outside this range
ax.grid(True)
ax.legend(loc='upper left')

# ============================================================
# Animation update function — called repeatedly by FuncAnimation
# ============================================================
def update(frame):
    """Read one line from UART, parse, update plot."""
    try:
        if ser.in_waiting > 0:
            raw = ser.readline().decode('utf-8', errors='ignore').strip()

            if not raw:
                return line,

            # Parse — Task 2 sends a single integer: e.g. "27"
            values = raw.split(',')
            temp_val = int(values[0])

            # Append data
            time_axis.append(counter[0])
            temp_data.append(temp_val)
            counter[0] += 1

            # Keep only last NUM_SAMPLES points
            if len(time_axis) > NUM_SAMPLES:
                time_axis.pop(0)
                temp_data.pop(0)

            # Update plot line
            line.set_data(time_axis, temp_data)
            ax.set_xlim(
                time_axis[0] if time_axis else 0,
                time_axis[-1] + 1 if time_axis else NUM_SAMPLES
            )

            # Auto-scale y-axis with a small margin
            if temp_data:
                margin = 5
                ax.set_ylim(min(temp_data) - margin, max(temp_data) + margin)

    except ValueError:
        print(f"[WARN] Could not parse line: '{raw}'")
    except Exception as e:
        print(f"[ERROR] {e}")

    return line,

# ============================================================
# Run animation  (interval = ms between frames)
# ============================================================
ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

plt.tight_layout()
plt.show()

# Close serial port when window is closed
ser.close()