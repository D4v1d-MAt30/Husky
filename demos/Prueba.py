import numpy as np
from RRT import rrt_algorithm

# Size of the point cloud
width, height = 100, 100

# Generate the point cloud (x, y coordinates)
x, y = np.meshgrid(np.arange(width), np.arange(height))
x = x.flatten()
y = y.flatten()

# Create a set to store obstacle coordinates
obstacles = set()

# Size of each obstacle block
block_size = 10

# Number of blocks to create
num_blocks = 20

np.random.seed(42)

for _ in range(num_blocks):
    # Top-left coordinate of the block
    start_x = np.random.randint(0, width - block_size)
    start_y = np.random.randint(0, height - block_size)

    # Add all points in the block to the obstacle set
    for dx in range(block_size):
        for dy in range(block_size):
            obstacles.add((start_x + dx, start_y + dy))

# Convert obstacles to a set of indices
points = list(zip(x, y))

# Separate free points and obstacle points
free_points = np.array([point for point in points if point not in obstacles])
obstacle_points = np.array([point for point in points if point in obstacles])

indices = np.random.choice(len(free_points), size=2, replace=False)
start_point = free_points[indices[0]]
goal_point = free_points[indices[1]]

print("Start:", start_point, "Goal:", goal_point)

max_iterations = 10000
incremental_distance = 2

path_nodes = rrt_algorithm(start_point, goal_point, incremental_distance, max_iterations,
                           free_points, obstacle_points, threshold=1, goal_threshold=0.25)