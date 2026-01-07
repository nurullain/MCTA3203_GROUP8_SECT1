# ===================== LIBRARIES =====================
import serial                          # Serial communication with ESP32
import matplotlib.pyplot as plt        # Plotting graphs
import matplotlib.animation as animation  # Real-time animation support
from collections import deque          # Efficient fixed-length data buffers

# ================= USER SETTINGS =================
PORT = 'COM10'        # 🔴 Serial port where ESP32 is connected (change if needed)
BAUD = 115200         # Baud rate (must match ESP32 Serial.begin)
MAX_POINTS = 60       # Number of data points shown on graph
# Fewer points = faster rendering
# =================================================

# ================= SERIAL SETUP =================
# Open serial port in non-blocking mode (timeout=0)
ser = serial.Serial(PORT, BAUD, timeout=0)

# ================= DATA BUFFERS =================
# Deques automatically discard old data when full
servo_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)  # Servo angle data
led_data   = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)  # LED brightness data
ldr_data   = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)  # LDR sensor data

# ================= PLOT SETUP =================
fig, ax = plt.subplots()               # Create plot window and axes

ax.set_xlim(0, MAX_POINTS)             # X-axis = number of samples
ax.set_ylim(0, 200)                    # Y-axis range (0–200)
ax.set_title("ESP32 Real-Time Sensor Graph")
ax.set_xlabel("Samples")
ax.set_ylabel("Value")

# Create three plot lines (initially empty)
line_servo, = ax.plot([], [], label="Servo")
line_led,   = ax.plot([], [], label="LED")
line_ldr,   = ax.plot([], [], label="LDR")

ax.legend(loc="upper right")            # Show legend

# ================= UPDATE FUNCTION =================
# This function is called repeatedly by FuncAnimation
def update(frame):
    # Read all available serial data
    while ser.in_waiting:
        try:
            # Read one line from serial, decode to string
            line = ser.readline().decode().strip()

            if line:
                # Expecting: servo,led,ldr
                values = line.split(",")

                if len(values) == 3:
                    # Append new values to buffers
                    servo_data.append(int(values[0]))
                    led_data.append(int(values[1]))
                    ldr_data.append(int(values[2]))
        except:
            # Ignore corrupted or incomplete serial data
            pass

    # Update plot data
    line_servo.set_data(range(len(servo_data)), servo_data)
    line_led.set_data(range(len(led_data)), led_data)
    line_ldr.set_data(range(len(ldr_data)), ldr_data)

    # Return updated lines for blitting
    return line_servo, line_led, line_ldr

# ================= ANIMATION =================
# FuncAnimation repeatedly calls update()
ani = animation.FuncAnimation(
    fig,
    update,
    interval=15,   # Refresh rate in milliseconds (fast update)
    blit=True      # Redraw only changed parts (better performance)
)

# ================= START PROGRAM =================
plt.show()           # Display the graph window
ser.close()          # Close serial port when window is closed
