import serial
import matplotlib
matplotlib.use('TkAgg')  # Force pop-up plot window
import matplotlib.pyplot as plt

# --- Serial Setup ---
arduino = serial.Serial(port='COM5', baudrate=9600, timeout=1)
print("Connected to:", arduino.name)


# --- Plot Setup ---
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], 'bo', markersize=3)

ax.set_xlabel("X Acceleration")
ax.set_ylabel("Y Acceleration")
ax.set_title("Real-Time MPU6050 X-Y Motion")
ax.grid(True)

x_data, y_data = [], []

arduino.reset_input_buffer()
print("Reading serial data... Press Ctrl + C to stop.\n")


# --- Real-time Loop ---
while True:
    try:
        line_raw = arduino.readline().decode(errors='ignore').strip()
        if not line_raw:
            continue
    try:
        ax_val, ay_val = map(int, parts)
    except ValueError:
        continue

    # Update data
    x_data.append(ax_val)
    y_data.append(ay_val)

    # Keep only latest 100 samples
    if len(x_data) > 100:
        x_data.pop(0)
        y_data.pop(0)

    # Update scatter line
    line.set_data(x_data, y_data)
    ax.relim()
    ax.autoscale_view()

    # Refresh plot
    plt.draw()
    plt.pause(0.01)

except KeyboardInterrupt:
print("\nExiting...")
arduino.close()
break