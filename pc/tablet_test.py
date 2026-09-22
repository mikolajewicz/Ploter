from pynput.mouse import Controller
import time

mouse = Controller()

SCREEN_WIDTH = 1536
SCREEN_HEIGHT = 864

while True:
    x, y = mouse.position

    nx = x / (SCREEN_WIDTH - 1)
    ny = y / (SCREEN_HEIGHT - 1)

    print(f"X = {nx:.3f}, Y = {ny:.3f}")

    time.sleep(0.1)