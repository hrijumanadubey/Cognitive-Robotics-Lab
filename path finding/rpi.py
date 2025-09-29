lab9
import numpy as np
import matplotlib.pyplot as plt
import heapq
import serial
import time
import math

# === CONFIG ===
MAP_FILE = "lab8_map.csv"
START = (10, 10)
GOAL = (15, 18)
PORT = "/dev/ttyACM0"
BAUD = 9600
CELL_SIZE = 20
MOVE_DELAY = 1.0

# === LOAD MAP ===
grid = np.loadtxt(MAP_FILE, delimiter=",")
rows, cols = grid.shape

# === HELPER FUNCTIONS ===
def heuristic(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def get_neighbors(node):
    neighbors = []
    directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    for dr, dc in directions:
        nr, nc = node[0]+dr, node[1]+dc
        if 0 <= nr < rows and 0 <= nc < cols:
            if grid[nr][nc] == 0:
                neighbors.append((nr,nc))
    return neighbors

def a_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start:0}
    f_score = {start:heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for neighbor in get_neighbors(current):
            cost = 1 if abs(neighbor[0]-current[0]) + abs(neighbor[1]-current[1]) == 1 else 1.414
            tentative_g = g_score[current] + cost
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

def get_angle(curr, next_cell):
    dy = next_cell[0]-curr[0]
    dx = next_cell[1]-curr[1]
    angle = math.degrees(math.atan2(dy, dx))
    return angle

# === CONNECT TO ARDUINO ===
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

# === RUN A* ===
path = a_star(START, GOAL)
if path is None:
    print("No path found!")
    exit()

# === SEND PATH COMMANDS ===
current_angle = 0
for i in range(1, len(path)):
    curr = path[i-1]
    next_cell = path[i]
    target_angle = get_angle(curr, next_cell)
    rotate_angle = target_angle - current_angle
    current_angle = target_angle

    # Rotate
    ser.write(f"ROTATE {int(rotate_angle)}\n".encode())
    time.sleep(MOVE_DELAY)
    # Move forward 1 cell
    ser.write(f"MOVE {CELL_SIZE}\n".encode())
    time.sleep(MOVE_DELAY)

# === VISUALIZE PATH ===
for step in path:
    grid[step[0]][step[1]] = 2

plt.figure(figsize=(6,6))
cmap = plt.cm.get_cmap("gray_r", 3)
plt.imshow(grid, cmap=cmap, origin="lower")
plt.scatter(START[1], START[0], c='green', marker='o', label='Start')
plt.scatter(GOAL[1], GOAL[0], c='red', marker='x', label='Goal')
plt.legend()
plt.title("Lab 9: A* Path + Smart Car Commands")
plt.show()
