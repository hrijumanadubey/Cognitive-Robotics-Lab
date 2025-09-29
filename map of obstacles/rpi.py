lab8
import serial
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

PORT = "/dev/ttyACM0"   # Arduino port
BAUD = 9600
CELL_SIZE = 20
MAX_DISTANCE_CM = 200

# Grid setup
num_cells_side = int(MAX_DISTANCE_CM / CELL_SIZE)
GRID_WIDTH = num_cells_side * 2 + 1
GRID_HEIGHT = num_cells_side * 2 + 1
carX = GRID_WIDTH // 2
carY = GRID_HEIGHT // 2
grid = np.zeros((GRID_HEIGHT, GRID_WIDTH))

ser = serial.Serial(PORT, BAUD, timeout=1)

plt.ion()
fig, ax = plt.subplots()

try:
    while True:
        line = ser.readline().decode().strip()
        if not line:
            continue
        try:
            obsX_str, obsY_str, distance_str = line.split(",")
            obsX = int(obsX_str)
            obsY = int(obsY_str)
            distance = int(distance_str)

            if distance < 2 or distance > MAX_DISTANCE_CM:
                continue

            gridX = carX + obsX
            gridY = carY + obsY
            if 0 <= gridX < GRID_WIDTH and 0 <= gridY < GRID_HEIGHT:
                grid[gridY][gridX] = 1

            ax.clear()
            ax.imshow(grid, cmap="gray_r", origin="lower")
            ax.plot(carX, carY, "ro")
            ax.set_title("Lab 8: Single-Position Occupancy Grid")
            plt.draw()
            plt.pause(0.05)
        except ValueError:
            continue
except KeyboardInterrupt:
    np.savetxt("lab8_map.csv", grid, fmt='%d', delimiter=",")
    print("Map saved to lab8_map.csv")
