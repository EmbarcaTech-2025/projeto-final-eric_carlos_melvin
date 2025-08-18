import tkinter as tk
from tkinter import ttk
import serial
import threading
import re

# Serial port configuration
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# Global variables to store Roll, Pitch, and Yaw for each MPU6050
mpu_data = {
    0: {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
    1: {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
    2: {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
}

# Regular expression to parse the serial data
data_pattern = re.compile(r"\[MPU6050 #(\d)\] Roll: ([\d\.\-]+)°, Pitch: ([\d\.\-]+)°, Yaw: ([\d\.\-]+)°")

# Function to read data from the serial port
def read_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        while True:
            line = ser.readline().decode("utf-8").strip()
            match = data_pattern.match(line)
            if match:
                mpu_id = int(match.group(1))
                roll = float(match.group(2))
                pitch = float(match.group(3))
                yaw = float(match.group(4))
                mpu_data[mpu_id]["Roll"] = roll
                mpu_data[mpu_id]["Pitch"] = pitch
                mpu_data[mpu_id]["Yaw"] = yaw
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")

# Function to update the GUI with the latest data
def update_gui():
    for mpu_id, data in mpu_data.items():
        roll_labels[mpu_id].config(text=f"Roll: {data['Roll']:.2f}°")
        pitch_labels[mpu_id].config(text=f"Pitch: {data['Pitch']:.2f}°")
        yaw_labels[mpu_id].config(text=f"Yaw: {data['Yaw']:.2f}°")
    root.after(100, update_gui)

# Create the GUI
root = tk.Tk()
root.title("MPU6050 Orientation Visualizer")

frame = ttk.Frame(root, padding="10")
frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

roll_labels = {}
pitch_labels = {}
yaw_labels = {}

for mpu_id in range(3):
    mpu_frame = ttk.LabelFrame(frame, text=f"MPU6050 #{mpu_id}", padding="10")
    mpu_frame.grid(row=mpu_id, column=0, padx=10, pady=10, sticky=(tk.W, tk.E))
    
    roll_labels[mpu_id] = ttk.Label(mpu_frame, text="Roll: 0.00°")
    roll_labels[mpu_id].grid(row=0, column=0, sticky=tk.W)
    
    pitch_labels[mpu_id] = ttk.Label(mpu_frame, text="Pitch: 0.00°")
    pitch_labels[mpu_id].grid(row=1, column=0, sticky=tk.W)
    
    yaw_labels[mpu_id] = ttk.Label(mpu_frame, text="Yaw: 0.00°")
    yaw_labels[mpu_id].grid(row=2, column=0, sticky=tk.W)

# Start the serial reading thread
serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

# Start the GUI update loop
update_gui()

# Run the GUI event loop
root.mainloop()