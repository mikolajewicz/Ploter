import serial
import time
from pynput.mouse import Controller

PORT = "COM5"   # tutaj później wpiszesz port ESP32
BAUD = 115200

SCREEN_WIDTH = 1920
SCREEN_HEIGHT = 1080

ser = serial.Serial(PORT, BAUD, timeout=1)

mouse = Controller()

time.sleep(2)

while True:
    x, y = mouse.position

    nx = x / SCREEN_WIDTH
    ny = y / SCREEN_HEIGHT

    nx = max(0.0, min(1.0, nx))
    ny = max(0.0, min(1.0, ny))

    command = f"tablet {nx:.4f} {ny:.4f}\n"

    ser.write(command.encode())

    print(command.strip())

    time.sleep(0.01)